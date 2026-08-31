# One obstacle as a single convex polytope (3D convex hull of its cluster
# after depth completion), in the header frame. Vertices are unordered;
# faces are triangles indexing into them. Empty object_id (all zeros) =
# a detection not yet bound to a track.
unique_identifier_msgs/UUID object_id
geometry_msgs/Point centroid       # polytope centroid
geometry_msgs/Vector3 velocity     # WORLD-frame linear velocity (tracked only; zero for detections)
geometry_msgs/Point[] vertices
uint32[] triangles                 # flat triples of vertex indices
float32 height                     # z extent [m]
bool coasting                      # true = no detection this frame; hull carried from the last one
