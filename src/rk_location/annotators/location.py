from timeit import default_timer

import numpy
import open3d as o3d
import py_trees
import robokudo.types
import robokudo.annotators.core
import robokudo.semantic_map
import robokudo.utils.transform
from robokudo.cas import CASViews
from robokudo.utils.module_loader import ModuleLoader
import open3d as o3d




def get_obb_from_semantic_map_region(
        region: robokudo.semantic_map.SemanticMapEntry) -> o3d.geometry.OrientedBoundingBox:
    """
    Instantiate an Open3D OrientedBoundingBox that represents a Semantic Map region.
    It can be used to crop pointclouds. It doesn't carry any frame information, so
    make sure to translate/rotate it to the desired coordinate frame if necessary.

    :param region: A semantic map entry that defines the region of interest in 3D
    :return: The bounding box with the pose and extent defined by region
    """
    obb = o3d.geometry.OrientedBoundingBox(
        center=numpy.array([region.position_x, region.position_y, region.position_z]),
        R=robokudo.utils.transform.get_rotation_matrix_from_q(
            numpy.array([region.orientation_x, region.orientation_y,
                         region.orientation_z, region.orientation_w])),
        extent=numpy.array([region.x_size, region.y_size, region.z_size])
    )
    return obb


class RegionFilter(robokudo.annotators.core.ThreadedAnnotator):
    """
    The RegionFilter can be used to filter point clouds based on a environment model based on different
    regions. These regions are collected in a 'SemanticMap' which has one 'SemanticMapEntry' per region of interest.
    Semantics to these regions are linked by referencing well-known names from your URDF and/or knowledge base.
    """

    class Descriptor(robokudo.annotators.core.BaseAnnotator.Descriptor):
        class Parameters:
            def __init__(self):
                self.world_frame_name = "map"
                self.semantic_map_ros_package = "robokudo"
                self.semantic_map_name = "semantic_map_iai_kitchen"  # should be in descriptors/semantic_maps/
                self.desired_regions = ["C", "D"]

        parameters = Parameters()  # overwrite the parameters explicitly to enable auto-completion

    def __init__(self, name="RegionFilter", descriptor=Descriptor()):
        super().__init__(name=name, descriptor=descriptor)
        self.semantic_map = None
        self.load_semantic_map()

    def load_semantic_map(self) -> None:
        module_loader = ModuleLoader()
        self.semantic_map = module_loader.load_semantic_map(self.descriptor.parameters.semantic_map_ros_package,
                                                            self.descriptor.parameters.semantic_map_name)

    def compute(self):
        start_timer = default_timer()
        cloud = self.get_cas().get(CASViews.CLOUD)

        self.load_semantic_map()
        self.semantic_map.publish_visualization_markers()

        active_regions = self.semantic_map.entries
        object_hypotheses = self.get_cas().filter_annotations_by_type(robokudo.types.scene.ObjectHypothesis)
        # TODO Filter active regions by either:
        #  - QUERY
        #  - FRUSTUM CULLING

        cam_to_world_transform = self.get_cas().get(robokudo.cas.CASViews.VIEWPOINT_CAM_TO_WORLD)

        cam_to_world_transform_matrix = robokudo.utils.transform.get_transform_matrix_from_q(
            cam_to_world_transform.rotation,
            cam_to_world_transform.translation)
        world_to_cam_transform_matrix = numpy.linalg.inv(cam_to_world_transform_matrix)

        visualized_geometries = []
        filtered_indices = set()

        self.rk_logger.debug(f"Analyzing {len(active_regions.keys())}")

        for key, region in active_regions.items():
            if key in self.descriptor.parameters.desired_regions:
                assert (isinstance(region, robokudo.semantic_map.SemanticMapEntry))

                # If the Semantic Map Region is in camera coordinates, we can keep the OBB as-is.
                # Otherwise, check if and how we shall transform it
                if region.frame_id == self.descriptor.parameters.world_frame_name:
                    obb = get_obb_from_semantic_map_region(region)
                    # Use the cam to world transform in the CAS.
                    # The benefit is, that this transform can also be recorded for our stored percepts in mongo
                    # and therefore allows using the region filter even without having live tf data
                    obb.rotate(robokudo.utils.transform.get_rotation_from_transform_matrix(world_to_cam_transform_matrix),
                               center=(0, 0, 0))
                    obb.translate(
                        robokudo.utils.transform.get_translation_from_transform_matrix(world_to_cam_transform_matrix))

                else:
                    # TODO Handle non-empty frames by using TF to transform them properly

                    obb = get_obb_from_semantic_map_region(region)

                # To avoid copying the whole points per region into a new cloud, we'll only collect and save
                # the indices of matching points
                region_indices = obb.get_point_indices_within_bounding_box(cloud.points)
                #filtered_indices.update(region_indices)
                print("Region indices ", len(region_indices))

                visualized_geometries.append({"name": region.name, "geometry": obb})
                for object_hypothesis in object_hypotheses:
                    total_indices = len(object_hypothesis.point_indices[0])
                    print("Total indices: ",total_indices)
                    index_inside_region = 0
                    for index in object_hypothesis.point_indices[0]:
                        if index in region_indices:
                            index_inside_region +=1
                    print("Number of index inside region: ",index_inside_region)
                    percentage_indices_inside = (index_inside_region/total_indices) * 100
                    print("Percentage: ",percentage_indices_inside)
                    #print("Object Name: ", object_hypothesis.annotations.classname)
                    if percentage_indices_inside >= 50:
                        # Create a Text annotation object
                        location_annotation = robokudo.types.annotation.LocationAnnotation()
                        # Set the text to the text object
                        location_annotation.L = key
                        object_hypothesis.annotations.append(location_annotation)

        # Place the filtered PointCloud into the CAS, overwriting the previous one
        filtered_cloud = cloud.select_by_index(list(filtered_indices))
        self.get_cas().set(CASViews.CLOUD, filtered_cloud)

        visualized_geometries.append({"name": "filtered cloud", "geometry": filtered_cloud})

        world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        visualized_geometries.append(
            {"name": "world_frame", "geometry": world_frame.transform(world_to_cam_transform_matrix)})
        self.get_annotator_output_struct().set_geometries(visualized_geometries)

        end_timer = default_timer()
        self.feedback_message = f'Processing took {(end_timer - start_timer):.4f}s'
        return py_trees.Status.SUCCESS