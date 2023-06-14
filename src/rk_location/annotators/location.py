from timeit import default_timer

import numpy as np
import py_trees
import copy
import robokudo.annotators
import robokudo.semantic_map
from robokudo.cas import CASViews
from robokudo.utils.module_loader import ModuleLoader



class LocationAnnotator(robokudo.annotators.core.ThreadedAnnotator):
    class Descriptor(robokudo.annotators.core.BaseAnnotator.Descriptor):
        class Parameters:
            def __init__(self):
                self.world_frame_name = "map"
                self.semantic_map_ros_package = "robokudo"
                self.semantic_map_name = "semantic_map_iai_kitchen"  # should be in descriptors/semantic_maps/
                self.region_to_consider = [['B',0.18,2.52],['A',0.18,2.72],['C',0.18,2.92]]

        parameters = Parameters()  # overwrite the parameters explicitly to enable auto-completion

    def __init__(self, name="LocationAnnotator",descriptor=Descriptor()):
        """
        Default construction. Minimal one-time init!
        """
        super().__init__(name=name, descriptor=descriptor)
        self.load_semantic_map()
        self.semantic_map = None

    def load_semantic_map(self) -> None:
        module_loader = ModuleLoader()
        self.semantic_map = module_loader.load_semantic_map(self.descriptor.parameters.semantic_map_ros_package,
                                                            self.descriptor.parameters.semantic_map_name)

    def consider_regions(self,active_region,region_to_consider_list):
        temp = copy.deepcopy(active_region)
        region_list = []
        for key, region in temp.items():
            assert (isinstance(region, robokudo.semantic_map.SemanticMapEntry))
            for considered_region in region_to_consider_list:
                region.y_size = considered_region[1]
                region.position_y = considered_region[2]
                region_list.append(temp)
        return region_list


    def compute(self):
        start_timer = default_timer()

        cloud = self.get_cas().get(CASViews.CLOUD)
        self.load_semantic_map()
        self.semantic_map.publish_visualization_markers()
        active_regions = self.semantic_map.entries
        region_list = self.consider_regions(active_regions, self.descriptor.parameters.region_to_consider)


        self.rk_logger.info(f"Cloud size is: {len(cloud.points)}")
        object_hypotheses = self.get_cas().filter_annotations_by_type(robokudo.types.scene.ObjectHypothesis)
        for object_hypothesis in object_hypotheses:
            if object_hypothesis.roi.mask is None:
                continue

            roi = object_hypothesis.roi.roi
            print("ye walla",object_hypothesis.points)

            object_position = np.arange(roi.pos.y,roi.pos.y + roi.height)[:, np.newaxis] + \
                              np.arange(roi.pos.x, roi.pos.x + roi.width)
            print(object_position)
            for count,region_dic in enumerate(region_list):
                for key, region in region_dic.items():
                    assert (isinstance(region, robokudo.semantic_map.SemanticMapEntry))
                    min_x = abs(region.position_x - region.x_size)
                    max_x = abs(region.position_x + region.x_size)
                    min_y = abs(region.position_y - region.y_size)
                    max_y = abs(region.position_y + region.y_size)
                    print(min_x,min_y,min_x,max_y)
                    if min_x < object_position[1].all() < max_x and min_y < object_position[0].all() < max_y:
                        # Create a Text annotation object
                        text_annotation = robokudo.types.annotation.Text()
                        # Set the text to the text object
                        text_annotation.text = self.descriptor.parameters.region_to_consider[count][0]
                        object_hypothesis.annotations.append(text_annotation)
                        print("Region of Object hyphothesis: ", text_annotation.text)

        end_timer = default_timer()
        self.feedback_message = f'Processing took {(end_timer - start_timer):.4f}s'
        return py_trees.Status.SUCCESS
