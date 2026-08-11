"""First geodesic experiment with a bent path over anchors."""

from geodesic.core.manifold import Manifold
from geodesic.core.measurements import distortion, path_length


def run() -> dict[str, float]:
    manifold = Manifold(dimensions=2)
    manifold.add_point("start", (0, 0))
    manifold.add_point("mid", (1, 0))
    manifold.add_point("end", (1, 1))
    path = ["start", "mid", "end"]
    return {"path_length": path_length(manifold, path), "distortion": distortion(manifold, path)}


if __name__ == "__main__":
    print(run())
