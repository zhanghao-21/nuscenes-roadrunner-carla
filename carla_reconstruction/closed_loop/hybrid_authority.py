"""CARLA/SUMO actor-authority helpers without simulator dependencies."""

from collections.abc import Mapping


STATIC_AUTHORITY = "carla_static"


def configured_carla_actor_specs(config, ego_spec):
    """Return ego, critical, and static CARLA specs with stable precedence."""
    if not isinstance(config, Mapping):
        raise ValueError("hybrid configuration must be an object")
    if not isinstance(ego_spec, Mapping):
        raise ValueError("ego specification must be an object")

    result = []
    seen = set()

    def append_spec(value, group):
        if not isinstance(value, Mapping):
            raise ValueError("%s entries must be objects" % group)
        spec = dict(value)
        raw_track_id = spec.get("track_id")
        if raw_track_id is None or isinstance(raw_track_id, bool):
            raise ValueError("%s entries require track_id" % group)
        track_id = str(raw_track_id).strip()
        if not track_id:
            raise ValueError("%s entries require track_id" % group)
        spec["track_id"] = track_id
        if track_id in seen:
            raise ValueError("duplicate CARLA-authority track_id: %s" % track_id)
        seen.add(track_id)
        result.append(spec)

    append_spec(ego_spec, "ego")
    critical = config.get("critical_actors", [])
    static = config.get("static_actors", [])
    if not isinstance(critical, (list, tuple)):
        raise ValueError("critical_actors must be a list")
    if not isinstance(static, (list, tuple)):
        raise ValueError("static_actors must be a list")
    for spec in critical:
        append_spec(spec, "critical_actors")
    explicit_actor_ids = set(seen)
    for value in static:
        if not isinstance(value, Mapping):
            raise ValueError("static_actors entries must be objects")
        spec = dict(value)
        raw_track_id = spec.get("track_id")
        if raw_track_id is None or isinstance(raw_track_id, bool):
            raise ValueError("static_actors entries require track_id")
        track_id = str(raw_track_id).strip()
        if not track_id:
            raise ValueError("static_actors entries require track_id")
        # Explicit ego/critical authority wins over automatic static authority.
        if track_id in explicit_actor_ids:
            continue
        authority = spec.setdefault("authority", STATIC_AUTHORITY)
        if authority != STATIC_AUTHORITY:
            raise ValueError(
                "static_actors authority must be %s" % STATIC_AUTHORITY)
        append_spec(spec, "static_actors")
    return result


def actor_spawn_policy(track_id, authority):
    """Return ``(role_name, physics_enabled)`` for a CARLA-authority actor."""
    authority = str(authority)
    if track_id == "ego":
        role_name = "hero"
    elif authority == STATIC_AUTHORITY:
        role_name = "static_recorded"
    else:
        role_name = "critical_actor"
    physics = authority not in ("carla_replay", STATIC_AUTHORITY)
    return role_name, physics
