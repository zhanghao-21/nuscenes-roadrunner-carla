import torch
import torch.nn as nn
import numpy as np


from .hrt_s import HRT, HRTNoEdgeInit

class MLP(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dims=(1024, 512), activation='relu'):
        super(MLP, self).__init__()
        dims = []
        dims.append(input_dim)
        dims.extend(hidden_dims)
        dims.append(output_dim)
        layers = []
        for i in range(len(dims) - 1):
            layers.append(nn.Linear(dims[i], dims[i + 1]))
            if i < len(dims) - 2:
                if activation == 'relu':
                    layers.append(nn.ReLU(inplace=True))
                elif activation == 'sigmoid':
                    layers.append(nn.Sigmoid())
        self.layers = nn.Sequential(*layers)

    def forward(self, x):
        x = self.layers(x)
        return x


class PositionalAgentEncoding(nn.Module):
    def __init__(self, d_model, dropout=0.1, max_t_len=200, concat=True):
        super(PositionalAgentEncoding, self).__init__()
        self.dropout = nn.Dropout(p=dropout)
        self.concat = concat
        self.d_model = d_model
        if concat:
            self.fc = nn.Linear(2 * d_model, d_model)

        pe = self.build_pos_enc(max_t_len)
        self.register_buffer('pe', pe)

    def build_pos_enc(self, max_len):
        pe = torch.zeros(max_len, self.d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, self.d_model, 2).float() * (-np.log(10000.0) / self.d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        return pe
    
    def get_pos_enc(self, num_t, num_a, t_offset):
        pe = self.pe[t_offset: num_t + t_offset, :]
        pe = pe[None].repeat(num_a, 1, 1)
        return pe

    def get_agent_enc(self, num_t, num_a, a_offset):
        ae = self.ae[a_offset: num_a + a_offset, :]
        ae = ae.repeat(num_t, 1, 1)
        return ae

    def forward(self, x, num_a, t_offset=0):
        num_t = x.shape[1]
        pos_enc = self.get_pos_enc(num_t, num_a, t_offset)  # (N, T, D)
        if self.concat:
            feat = [x, pos_enc]
            x = torch.cat(feat, dim=-1)
            x = self.fc(x)
        else:
            x += pos_enc
        return self.dropout(x)  # (N, T, D)


class Decoder(nn.Module):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.multiplier = 3  # update to 3 to match concatenated feature of n_final, cur_proj, and planned_feat_all.
        # This MLP predicts the trajectory. Its output is reshaped to (future_length, 2).
        self.decoder_mlp = MLP(
            args.model_dim * self.multiplier,
            args.future_length * 2,
            hidden_dims=(
                args.decoder_hidden_dim,
                args.decoder_hidden_dim // 2
            )
        )
        # A separate linear layer to produce a scalar logit (confidence) per head.
        self.conf_layer = nn.Linear(args.model_dim * self.multiplier, 1)

    def forward(self, final_feature, cur_location):
        # final_feature shape: (B*num_agents, 3*model_dim)
        traj = self.decoder_mlp(final_feature)
        traj = traj.view(-1, self.args.future_length, 2)
        # If prediction is absolute, adjust with the current location
        if not self.args.pred_rel:
            traj = traj + cur_location
        # Produce a confidence logit for this head.
        logit = self.conf_layer(final_feature)  # shape: (B*num_agents, 1)
        return traj, logit



# Replace your current Decoder with a GRU-based Decoder.
class DecoderGRU(nn.Module):
    def __init__(self, args):
        super(DecoderGRU, self).__init__()
        self.args = args
        self.future_length = args.future_length
        # Project final_feature (of size args.model_dim) to GRU hidden state dimension.
        self.fc_hidden = nn.Linear(args.model_dim, args.decoder_hidden_dim)
        # We'll use GRU with input_size equal to decoder_hidden_dim.
        self.gru = nn.GRU(input_size=args.decoder_hidden_dim, hidden_size=args.decoder_hidden_dim,
                          num_layers=1, batch_first=True)
        # Map GRU output to trajectory coordinates (2 dims) at each time step.
        self.out = nn.Linear(args.decoder_hidden_dim, 2)
        # Confidence layer (same as before).
        self.conf_layer = nn.Linear(args.model_dim, 1)

    def forward(self, final_feature, cur_location):
        # final_feature shape: (B*num_agents, d_final)
        # Compute initial hidden state for GRU.
        h0 = self.fc_hidden(final_feature)  # shape: (B*num_agents, decoder_hidden_dim)
        h0 = h0.unsqueeze(0)  # shape: (1, B*num_agents, decoder_hidden_dim)
        # Prepare GRU inputs: a zero tensor for each future step.
        decoder_input = torch.zeros(final_feature.size(0), self.future_length, self.args.decoder_hidden_dim,
                                      device=final_feature.device)
        # Decode the future trajectory.
        gru_out, _ = self.gru(decoder_input, h0)  # shape: (B*num_agents, future_length, decoder_hidden_dim)
        traj = self.out(gru_out)  # shape: (B*num_agents, future_length, 2)
        # If prediction is absolute, adjust with current location.
        if not self.args.pred_rel:
            traj = traj + cur_location  # cur_location expected shape: (B*num_agents, 1, 2)
        # Compute a confidence logit for this head.
        logit = self.conf_layer(final_feature)  # shape: (B*num_agents, 1)
        return traj, logit
    

class MARTS(nn.Module):
    def __init__(self, args):
        super(MARTS, self).__init__()
        self.args = args

        module_args = {
            'num_layers': 1,
            'num_heads': args.num_heads,
            'node_dim': args.model_dim,
            'node_hidden_dim': args.hidden_dim,
            'edge_dim': args.model_dim,
            'edge_hidden_dim_1': args.hidden_dim,
            'edge_hidden_dim_2': args.hidden_dim,
            'dropout': args.dropout,
            'aggregation': args.aggregation  # e.g., 'avg'
        }
        
        self.input_dim = len(args.inputs)
        self.input_fc = nn.Linear(self.input_dim, args.model_dim)
        self.input_fc2 = nn.Linear(args.model_dim * args.past_length, args.model_dim)
        
        self.pos_encoder = PositionalAgentEncoding(args.model_dim, 0.1, concat=True)
        
        # Only hyper-encoders are used.
        self.hyper_encoders = nn.ModuleList()
        for i in range(args.num_layers):
            if i == 0:
                self.hyper_encoders.append(HRT(**module_args, function_type=args.function_type))
            else:
                self.hyper_encoders.append(HRTNoEdgeInit(**module_args, function_type=args.function_type))
        
        # New layers for planned trajectory and current position projection.
        self.cur_fc = nn.Linear(2, args.model_dim)
        self.planned_fc = nn.Linear(2, args.model_dim)
        self.planned_pos_encoder = PositionalAgentEncoding(args.model_dim, 0.1, concat=True)
        
        # Create multiple prediction heads (decoders)
        self.decoders = nn.ModuleList([Decoder(args) for _ in range(args.sample_k)])
        # Alternatively, one could use DecoderGRU as in the commented code.
        
        
    def forward(self, x_abs, x_rel, planned_traj):
        batch_size, num_agents, length, _ = x_abs.shape
        # Compute current position for each agent (last observed position)
        cur_pos = x_abs[:, :, -1, :]  # shape: (B, N, 2)
                
        inputs = []
        if 'pos_x' in self.args.inputs and 'pos_y' in self.args.inputs:
            inputs.append(x_abs)
        if 'vel_x' in self.args.inputs and 'vel_y' in self.args.inputs:
            inputs.append(x_rel)
        
        inputs = torch.cat(inputs, dim=-1)
        inputs = inputs.view(batch_size * num_agents, length, -1).contiguous()
        
        inputs_fc = self.input_fc(inputs).view(batch_size * num_agents, length, self.args.model_dim)
        inputs_pos = self.pos_encoder(inputs_fc, num_a=batch_size * num_agents)
        inputs_pos = inputs_pos.view(batch_size, num_agents, length, self.args.model_dim)
        n_initial = self.input_fc2(inputs_pos.contiguous().view(batch_size, num_agents, length * self.args.model_dim))
        
        # Only using the hyper-encoder branch.
        n_group, e_group, G = n_initial, None, None
        for i in range(self.args.num_layers):
            n_group, e_group, G = self.hyper_encoders[i](n_group, e_group, G, return_edge=True)
        
        n_final = n_group  # shape: (B, N, model_dim)
        
        # Process current position for all agents.
        cur_proj = self.cur_fc(cur_pos)  # shape: (B, N, model_dim)
        
        # Process planned trajectory (available for target agent only).
        # planned_traj: (B, future_length, 2)
        planned_emb = self.planned_fc(planned_traj)  # (B, future_length, model_dim)
        # Use the batch size (B) for positional encoding instead of 1:
        planned_emb = self.planned_pos_encoder(planned_emb, num_a=planned_emb.size(0))
        planned_feat = planned_emb.mean(dim=1)  # (B, model_dim) via average pooling
        
        # Build a planned feature tensor for all agents:
        # For the target agent (index 0) use the computed planned_feat; for others, fill with zeros.
        planned_feat_all = torch.zeros(batch_size, num_agents, self.args.model_dim, device=x_abs.device)
        planned_feat_all[:, 0, :] = planned_feat
        
        # Concatenate n_final, cur_proj, and planned_feat_all.
        combined_feat = torch.cat([n_final, cur_proj, planned_feat_all], dim=-1)  # (B, N, 3*model_dim)
        
        # Flatten for decoder input.
        combined_feat = combined_feat.view(batch_size * num_agents, -1)
        # Also flatten current position for decoder adjustment.
        cur_pos_flat = cur_pos.view(batch_size * num_agents, 1, 2)
        
        # Collect predictions and logits from each decoder head.
        traj_list = []
        logit_list = []
        for decoder in self.decoders:
            traj, logit = decoder(combined_feat, cur_pos_flat)
            traj_list.append(traj.unsqueeze(1))   # shape: (B*num_agents, 1, future_length, 2)
            logit_list.append(logit.unsqueeze(1))   # shape: (B*num_agents, 1, 1)
        
        # Stack along the head dimension.
        trajs = torch.cat(traj_list, dim=1)   # shape: (B*num_agents, sample_k, future_length, 2)
        logits = torch.cat(logit_list, dim=1).squeeze(-1)   # shape: (B*num_agents, sample_k)
        probs = torch.softmax(logits, dim=1)  # shape: (B*num_agents, sample_k)
        
        # Reshape to separate batch and agent dimensions.
        trajs = trajs.view(batch_size, num_agents, self.args.sample_k, self.args.future_length, 2)
        probs = probs.view(batch_size, num_agents, self.args.sample_k)
        
        return trajs, probs, G
