"""Shared swarm algorithms plus catkin-generated configuration modules."""
from pkgutil import extend_path

# A catkin relay for another bundled package may put src first on sys.path.
# Keep the generated abc_swarm.cfg directory discoverable in either import order.
__path__ = extend_path(__path__, __name__)
