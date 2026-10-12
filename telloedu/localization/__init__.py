from .base import (PoseEstimate, LocalizationProvider, GroundTruthLocalization,
                   DeadReckoningLocalization, ArucoLocalization,
                   MissionPadLocalization, ExternalLocalization)

def create_localization(kind, drone=None, **kwargs):
    k=str(kind).lower()
    if k in ('ground_truth','gt'): return GroundTruthLocalization(drone)
    if k in ('dead_reckoning','dr'): return DeadReckoningLocalization(drone, **kwargs)
    if k in ('mission_pad','pad'): return MissionPadLocalization(drone)
    if k=='aruco': return ArucoLocalization(**kwargs)
    if k in ('slam','mocap','external'): return ExternalLocalization(name=k, **kwargs)
    raise ValueError(f'Unknown localization provider: {kind}')
