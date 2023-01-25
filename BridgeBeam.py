import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import NemAll_Python_Utility as AllplanUtil
import GeometryValidate as GeometryValidate
import StdReinfShapeBuilder.GeneralReinfShapeBuilder as GeneralShapeBuilder
import StdReinfShapeBuilder.LinearBarPlacementBuilder as LinearBarBuilder
import NemAll_Python_Reinforcement as Reinforcement
from StdReinfShapeBuilder.RotationAngles import RotationAngles
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties
from HandleService import HandleService
from StdReinfShapeBuilder.ConcreteCoverProperties import ConcreteCoverProperties
from StdReinfShapeBuilder.ReinforcementShapeProperties import ReinforcementShapeProperties

class CreateBridge():
    def __init__(self, doc):

        self.model_ele_list = []
        self.handle_list = []
        self.document = doc
        self.length = None
        self.height = None
        self.width = None
        self._topSH_width,_topSH_height = None
        self._botSH_width,_botSH_up_height,_botSH_low_height,_botSH_height = None
        self._hole_depth,_hole_height = None
        self._angleX,_angleY,_angleZ = None
        self._concrete_grade = None
        self._steel_grade = None
        self._bar_diameter,_bar_distance,_bar_height,_bar_depth = None
        self._concrete_cover = None
        self._bending_roller = None
        self._hook_length = None
        self._middle_width ,_middle_height = None
    def create(self, build_El):
        self.create_par(self,build_El)
        self.create_top(self, build_El)
        self.create_bot(self, build_El)
        self.create_holeAngle(self, build_El)
        self.create_beam(self, build_El)
        self.create_mid(self, build_El)
        self.create_conc(self, build_El)
        self.create_bar(self, build_El)
        self.create_B(build_El)
        self.reinforcement(build_El)
        self.create_handles12(self)
        self.create_handle34(self)
        self.create_handle5(self)


        AllplanBaseElements.ElementTransform(AllplanGeo.Vector3D(), self._angleX, self._angleY, self._angleZ,
                                             self.model_ele_list)

        rot_angles = RotationAngles(self._angleX, self._angleY, self._angleZ)
        HandleService.transform_handles(self.handle_list, rot_angles.get_rotation_matrix())

        return (self.model_ele_list, self.handle_list)
    def create_par(self,build_El):
        self.length = build_El.Length.value
        self.height = build_El.Height.value
        self.width = build_El.Width.value
        self._steel_grade = build_El.SteelGrade.value
        self._bending_roller = build_El.BendingRoller.value
        self._hook_length = build_El.HookLength.value
    def create_top(self, build_El):
        self._topSH_width = build_El.TopShWidth.value
        self._topSH_height = build_El.TopShHeight.value
    def create_mid(self, build_El):
        self._middle_width  = build_El.MiddleWidth.value
        self._middle_height = build_El.MiddleHeight.value
    def create_conc(self,build_El):
        self._concrete_grade = build_El.ConcreteGrade.value
        self._concrete_cover = build_El.ConcreteCover.value
    def create_bar(self,build_El):
        self._bar_diameter = build_El.BarDiameter.value
        self._bar_distance = build_El.BarDistance.value
        self._bar_height = build_El.BarHeight.value
        self._bar_depth = build_El.BarDepth.value
    def create_bot(self, build_El):
        self._botSH_width = build_El.BotShWidth.value
        self._botSH_up_height = build_El.BotShUpHeight.value
        self._botSH_low_height = build_El.BotShLowHeight.value
        self._botSH_height = self._botSH_up_height + self._botSH_low_height
    def create_holeAngle(self, build_El):
        self._hole_depth = build_El.HoleDepth.value
        self._hole_height = build_El.HoleHeight.value
        self._angleX = build_El.RotationAngleX.value
        self._angleY = build_El.RotationAngleY.value
        self._angleZ = build_El.RotationAngleZ.value
    def create_beam(self,build_El):
        self._beam_length = build_El.BeamLength.value
        self._beam_width = max(self._topSH_width, self._botSH_width)
        self._beam_height = build_El.BeamHeight.value

    def create_B(self, build_El):
        com_prop = AllplanBaseElements.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = build_El.Color3.value
        com_prop.Stroke = 1

        breps = AllplanGeo.BRep3DList()
        bottom_shelf = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D((self._beam_width - self._botSH_width) / 2., 0., 0.),
                                       AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)),
            self._botSH_width / 2., self._beam_length / 2., self._botSH_height)

        edges = AllplanUtil.VecSizeTList()
        edges.append(10)
        err, bottom_shelf = AllplanGeo.ChamferCalculus.Calculate(bottom_shelf, edges, 20., False)
        self.Geometry_err(err)
        breps.append(bottom_shelf)
        top_shelf = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(
            AllplanGeo.Point3D((self._beam_width - self._topSH_width) / 2., 0.,
                               self._beam_height - self._topSH_height), AllplanGeo.Vector3D(1, 0, 0),
            AllplanGeo.Vector3D(0, 0, 1)), self._topSH_width / 2., self._beam_length / 2., self._topSH_height)

        top_shelf_notch = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(
            AllplanGeo.Point3D((self._beam_width - self._topSH_width) / 2., 0., self._beam_height - 45.),
            AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), 60., self._beam_length / 2., 45.)
        err, top_shelf = AllplanGeo.MakeSubtraction(top_shelf, top_shelf_notch)
        self.Geometry_err(err)
        breps.append(top_shelf)
        middle_shelf = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(
                AllplanGeo.Point3D(0.0, 0.0, self._botSH_height),
                AllplanGeo.Vector3D(1, 0, 0),
                AllplanGeo.Vector3D(0, 0, 1),
            ),
            self.width / 2.0,
            self.length / 2.0,
            self._middle_height,
        )
        breps.append(middle_shelf)
        err, beam = AllplanGeo.MakeUnion(breps)
        self.Geometry_err(err)
        extraction = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(
                    (self.width - self._topSH_width) / 2.0, 0.0, self.height - 45.0
                ),
                AllplanGeo.Vector3D(1, 0, 0),
                AllplanGeo.Vector3D(0, 0, 1),
            ),
            60.0,
            self.length / 2.0,
            45.0,
        )
        breps.append(extraction)
        err, beam = AllplanGeo.MakeUnion(breps)
        self.Geometry_err(err)
        hole_self = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(
                AllplanGeo.Point3D(
                    0, build_El.HoleDepth.value, build_El.HoleHeight.value
                ),
                AllplanGeo.Vector3D(0, 0, 1),
                AllplanGeo.Vector3D(1, 0, 0),
            ),
            45.5,
            self.width,
        )
        breps.append(hole_self)
        err, beam = AllplanGeo.MakeUnion(breps)
        self.Geometry_err(err)
        extraction_parts = AllplanGeo.BRep3DList()
        notch_pol = AllplanGeo.Polyline3D()
        start_point = AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2., 0.,
                                         self._beam_height - self._topSH_height)
        notch_pol += start_point
        notch_pol += AllplanGeo.Point3D(
            (self.width - self._middle_width) / 2, 0, self._botSH_height
        )
        notch_pol += AllplanGeo.Point3D(
            (self.width - self._botSH_width) / 2, 0, 153
        )
        notch_pol += AllplanGeo.Point3D(-10, 0, 153)
        notch_pol += AllplanGeo.Point3D(-10, 0, self.height - 100)
        notch_pol += AllplanGeo.Point3D(
            (self.width - self._topSH_width) / 2, 0.0, self.height - 100
        )
        notch_pol += start_point
        path = AllplanGeo.Polyline3D()
        path += AllplanGeo.Point3D(0, 0, 0)
        path += AllplanGeo.Point3D(0, self.length / 2., 0)
        self.Geometry_err(err)
        err, notch = AllplanGeo.CreateSweptBRep3D(notch_pol, path, False, None)
        self.Geometry_err(err)
        err, notch = AllplanGeo.FilletCalculus3D.Calculate(notch, edges, 100., False)
        self.Geometry_err(err)
        extraction_parts.append(notch)
        breps = AllplanGeo.BRep3DList()


        path = AllplanGeo.Polyline3D()
        path += AllplanGeo.Point3D(0, 0, 0)
        path += AllplanGeo.Point3D(0, self._varying_start, 0) if build_El.CheckBoxV.value else AllplanGeo.Point3D(0,
                                                                                                                  self._beam_length / 2.,
                                                                                                                  0)

        err, notch = AllplanGeo.CreateSweptBRep3D(notch_pol, path, False, None)
        self.Geometry_err(err)
        edges = AllplanUtil.VecSizeTList()
        edges.append(3)
        edges.append(1)
        err, notch = AllplanGeo.FilletCalculus3D.Calculate(notch, edges, 100., False)
        self.Geometry_err(err)
        breps.append(notch)
        self.v_notches(self, build_El, notch_pol, breps, edges)
        self.siling_holes(self, beam, build_El, breps)
        self.results(self, beam, com_prop)

    def reinforcement(self, build_El):
        line_1 = AllplanGeo.Line3D(
            AllplanGeo.Point3D(
                (self.width - self._middle_width ) / 2, 0, self.height - self._topSH_height
            ),
            AllplanGeo.Point3D((self.width - self._topSH_height) / 2, 0, self.height - 100),
        )
        x = (self.width - self._topSH_width) / 2 + 60 + self._concrete_cover
        line_2 = AllplanGeo.Line3D(
            AllplanGeo.Point3D(x, 0, self.height - self._topSH_height),
            AllplanGeo.Point3D(x, 0, self.height),
        )
        err, intersection = AllplanGeo.IntersectionCalculus(line_1, line_2)
        if self.height - self._bar_depth < intersection.Z:
            build_El.BarDepth.value = self.height - intersection.Z
            self._bar_depth = build_El.BarDepth.value
        if self.height - self._bar_depth < intersection.Z:
            build_El.BarDepth.value = self.height - intersection.Z
            self._bar_depth = build_El.BarDepth.value

        model_angles = RotationAngles(90, -90, 0)

        shape_props = ReinforcementShapeProperties.rebar(
            self._bar_diameter,
            self._bending_roller,
            self._steel_grade,
            self._concrete_grade,
            Reinforcement.BendingShapeType.LongitudinalBar,
        )
        _concrete_cover_props = ConcreteCoverProperties.left_right_bottom(
            self._concrete_cover, self._concrete_cover, -self._concrete_cover
        )

        shape = GeneralShapeBuilder.create_longitudinal_shape_with_hooks(
            self._bar_height + self._bar_depth,
            model_angles,
            shape_props,
            _concrete_cover_props,
            -1,
            self._hook_length,
        )

        x = (self.width - self._topSH_width) / 2 + 60
        z = self.height - self._bar_depth
        from_point = AllplanGeo.Point3D(x, 0, z)
        to_point = AllplanGeo.Point3D(x, self.length, z)
        self.model_ele_list.append(
            LinearBarBuilder.create_linear_bar_placement_from_to_by_dist(
                0, shape, from_point, to_point, 0, 0, self.bar_spacing
            )
        )

        shape.Rotate(RotationAngles(0, 0, 180))
        x = self._topSH_width - 60.0
        from_point = AllplanGeo.Point3D(x, 0, z)
        to_point = AllplanGeo.Point3D(x, self.length, z)
        self.model_ele_list.append(
            LinearBarBuilder.create_linear_bar_placement_from_to_by_dist(
                0, shape, from_point, to_point, 0, 0, self.bar_spacing
            )
        )

    def v_notches(self, build_El, notch_pol, breps, edges):
        if build_El.CheckBoxV.value:
            profiles = []
            profiles.append(AllplanGeo.Move(notch_pol, AllplanGeo.Vector3D(0, self._varying_start, 0)))

            lines = []
            lines.append(AllplanGeo.Line3D(notch_pol.GetPoint(0), notch_pol.GetPoint(5)))
            lines.append(AllplanGeo.Line3D(notch_pol.GetPoint(1), notch_pol.GetPoint(2)))
            lines.append(AllplanGeo.Move(AllplanGeo.Line3D(notch_pol.GetPoint(0), notch_pol.GetPoint(1)),
                                         AllplanGeo.Vector3D((self._rib_thickness - self._varying_rib_thickness) / 2.,
                                                             0, 0)))
            intersections = [None, None]
            b, intersections[0] = AllplanGeo.IntersectionCalculusEx(lines[0], lines[2])
            b, intersections[1] = AllplanGeo.IntersectionCalculusEx(lines[1], lines[2])

            notch_pol = AllplanGeo.Polyline3D()
            start_point = AllplanGeo.Point3D((self._beam_width - self._varying_rib_thickness) / 2., self._varying_end,
                                             intersections[0].Z)
            notch_pol += start_point
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._varying_rib_thickness) / 2., self._varying_end,
                                            intersections[1].Z)
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._botSH_width) / 2., self._varying_end,
                                            self._botSH_low_height)
            notch_pol += AllplanGeo.Point3D(-10., self._varying_end, self._botSH_low_height)
            notch_pol += AllplanGeo.Point3D(-10., self._varying_end, self._beam_height - 100.)
            notch_pol += AllplanGeo.Point3D((self._beam_width - self._topSH_width) / 2., self._varying_end,
                                            self._beam_height - 100.)
            notch_pol += start_point
            path = AllplanGeo.Polyline3D()
            path += AllplanGeo.Point3D(0, self._varying_end, 0)
            path += AllplanGeo.Point3D(0, self._beam_length / 2., 0)

            err, notch = AllplanGeo.CreateSweptBRep3D(notch_pol, path, False, None)
            self.Geometry_err(err)
            err, notch = AllplanGeo.FilletCalculus3D.Calculate(notch, edges, 100., False)
            self.Geometry_err(err)
            breps.append(notch)

            profiles.append(notch_pol)
            path = AllplanGeo.Line3D(profiles[0].GetStartPoint(), profiles[1].GetStartPoint())

            err, notch = AllplanGeo.CreateRailSweptBRep3D(profiles, [path], True, False, False)

            edges = AllplanUtil.VecSizeTList()
            edges.append(11)
            edges.append(9)
            err, notch = AllplanGeo.FilletCalculus3D.Calculate(notch, edges, 100., False)
            self.Geometry_err(err)
            breps.append(notch)

    def siling_holes(self, beam, build_El, breps):
        sling_hole = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0, build_El.HoleDepth.value, build_El.HoleHeight.value),
                                       AllplanGeo.Vector3D(0, 0, 1), AllplanGeo.Vector3D(1, 0, 0)), 45.5,
            self._beam_width)
        breps.append(sling_hole)

        err, beam = AllplanGeo.MakeSubtraction(beam, breps)
        if not GeometryValidate.polyhedron(err):
            return

    def Geometry_err(err):
        if not GeometryValidate.polyhedron(err):
            return

    def results(self, beam, com_prop):
        plane = AllplanGeo.Plane3D(AllplanGeo.Point3D(self._beam_width / 2., 0, 0), AllplanGeo.Vector3D(1, 0, 0))
        err, beam = AllplanGeo.MakeUnion(beam, AllplanGeo.Mirror(beam, plane))
        if not GeometryValidate.polyhedron(err):
            return
        plane.Set(AllplanGeo.Point3D(0, self._beam_length / 2., 0), AllplanGeo.Vector3D(0, 1, 0))
        err, beam = AllplanGeo.MakeUnion(beam, AllplanGeo.Mirror(beam, plane))
        if not GeometryValidate.polyhedron(err):
            return
        self.model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop, beam))

    def create_handle12(self):
        handle1 = HandleProperties(
            "BeamLength",
            AllplanGeo.Point3D(0., self._beam_length, 0.),
            AllplanGeo.Point3D(0, 0, 0),
            [("BeamLength", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle1)
        handle2 = HandleProperties(
            "BeamHeight",
            AllplanGeo.Point3D(0., 0., self._beam_height),
            AllplanGeo.Point3D(0, 0, 0),
            [("BeamHeight", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle2)

    def create_handle34(self):
        handle3 = HandleProperties(
            "TopShWidth",
            AllplanGeo.Point3D(
                (self._beam_width - self._topSH_width) / 2. + self._topSH_width, 0., self._beam_height - 45.
            ),
            AllplanGeo.Point3D((self._beam_width - self._topSH_width) / 2., 0, self._beam_height - 45.),
            [("TopShWidth", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle3)
        handle4 = HandleProperties(
            "BotShWidth",
            AllplanGeo.Point3D(
                (self._beam_width - self._botSH_width) / 2. + self._botSH_width, 0., self._botSH_low_height
            ),

            AllplanGeo.Point3D((self._beam_width - self._botSH_width) / 2., 0, self._botSH_low_height),
            [("BotShWidth", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle4)

    def create_handle5(self):
        handle5 = HandleProperties(
            "RibThick",
            AllplanGeo.Point3D(
                (self._beam_width - self._rib_thickness) / 2. + self._rib_thickness, 0., self._beam_height / 2.
            ),
            AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2., 0, self._beam_height / 2.),
            [("RibThick", HandleDirection.point_dir)],
            HandleDirection.point_dir, True
        )
        self.handle_list.append(handle5)

def geometry_equality(self,notches,beam,err) :
    if GeometryValidate.polyhedron(err):
        edges = AllplanUtil.VecSizeTList()
        if self._rib_thickness == self._botSH_width:
            edges.append(0)
        elif self._rib_thickness == self._topSH_width:
            edges.append(1)
        else:
            edges.append(0)
            edges.append(2)
        err, notches = AllplanGeo.FilletCalculus3D.Calculate(notches, edges, 100., False)

        plane = AllplanGeo.Plane3D(AllplanGeo.Point3D(self._beam_width / 2., 0, 0), AllplanGeo.Vector3D(1, 0, 0))
        right_notch = AllplanGeo.Mirror(notches, plane)

        err, notches = AllplanGeo.MakeUnion(notches, right_notch)
        if not GeometryValidate.polyhedron(err):
            return

        err, beam = AllplanGeo.MakeSubtraction(beam, notches)
        if not GeometryValidate.polyhedron(err):
            return
def allplan_version(build_El, version):
    del build_El
    del version
    return True


def create_element(build_El, doc):
    element = CreateBridge(doc)
    return element.create(build_El)


def move_handle(build_El, handle_prop, input_pnt, doc):
    build_El.change_property(handle_prop, input_pnt)
    RibHeight_equality(handle_prop.handle_id, build_El)
    HoleHeight_equality(build_El, handle_prop.handle_id)
    return create_element(build_El, doc)


def RibHeight_equality(handle_id, build_El):
    if (handle_id == "BeamHeight"):
        build_El.RibHeight.value = build_El.BeamHeight.value - build_El.TopShHeight.value - build_El.BotShLowHeight.value - build_El.BotShUpHeight.value
        if (build_El.HoleHeight.value > build_El.BeamHeight.value - build_El.TopShHeight.value - 45.5):
            build_El.HoleHeight.value = build_El.BeamHeight.value - build_El.TopShHeight.value - 45.5


def HoleHeight_equality(build_El, handle_id):
    if (handle_id == "TopShWidth") or (handle_id == "BotShWidth") or (
            handle_id == "RibThick"):
        temp = min(build_El.TopShWidth.value, build_El.BotShWidth.value)
        RibThick_eq(build_El,temp)


def RibThick_eq(build_El,temp):
    if (build_El.RibThick.value >= temp - 100.):
        build_El.RibThick.value = temp - 100.
    if (build_El.RibThick.value <= build_El.VaryingRibThick.value):
        build_El.VaryingRibThick.value = build_El.RibThick.value - 20.
    elif (build_El.RibThick.value - 100. >= build_El.VaryingRibThick.value):
        build_El.VaryingRibThick.value = build_El.RibThick.value - 100.


def change_property(build_El, name, value):
    if name == "BeamHeight":
        change = value - build_El.TopShHeight.value - build_El.RibHeight.value - \
                 build_El.BotShUpHeight.value - build_El.BotShLowHeight.value
        print(change)
        change_prop_equality(change, build_El, value)
    else:
        if name == "TopShHeight" or name == "RibHeight":
            variation(build_El, name, value)
        if name == "BotShUpHeight" or name == "BotShLowHeight":
            variation_bot_height(build_El, name, value)
        if name == "HoleHeight" or name == "HoleDepth":
            variation_hole(build_El, name, value)

    return True


def change_prop_equality(change, build_El, value):
    if change < 0:
        change = abs(change)
        if build_El.TopShHeight.value > 320.:
            if build_El.TopShHeight.value - change < 320.:
                change -= build_El.TopShHeight.value - 320.
                build_El.TopShHeight.value = 320.
            else:
                build_El.TopShHeight.value -= change
                change = 0.
        if (change != 0) and (build_El.BotShUpHeight.value > 160.):
            if build_El.BotShUpHeight.value - change < 160.:
                change -= build_El.BotShUpHeight.value - 160.
                build_El.BotShUpHeight.value = 160.
            else:
                build_El.BotShUpHeight.value -= change
                change = 0.
        if (change != 0) and (build_El.BotShLowHeight.value > 153.):
            if build_El.BotShLowHeight.value - change < 153.:
                change -= build_El.BotShLowHeight.value - 153.
                build_El.BotShLowHeight.value = 153.
            else:
                build_El.BotShLowHeight.value -= change
                change = 0.
        if (change != 0) and (build_El.RibHeight.value > 467.):
            if build_El.RibHeight.value - change < 467.:
                change -= build_El.RibHeight.value - 467.
                build_El.RibHeight.value = 467.
            else:
                build_El.RibHeight.value -= change
                change = 0.
    else:
        build_El.RibHeight.value += change
    if value - build_El.TopShHeight.value - 45.5 < build_El.HoleHeight.value:
        build_El.HoleHeight.value = value - build_El.TopShHeight.value - 45.5


def variation(build_El, name, value):
    if name == "TopShHeight":
        build_El.BeamHeight.value = value + build_El.RibHeight.value + \
                                    build_El.BotShUpHeight.value + build_El.BotShLowHeight.value
    if name == "RibHeight":
        build_El.BeamHeight.value = value + build_El.TopShHeight.value + \
                                    build_El.BotShUpHeight.value + build_El.BotShLowHeight.value


def variation_bot_height(build_El, name, value):
    if name == "BotShUpHeight":
        build_El.BeamHeight.value = value + build_El.TopShHeight.value + \
                                    build_El.RibHeight.value + build_El.BotShLowHeight.value
        if value + build_El.BotShLowHeight.value + 45.5 > build_El.HoleHeight.value:
            build_El.HoleHeight.value = value + build_El.BotShLowHeight.value + 45.5
    if name == "BotShLowHeight":
        build_El.BeamHeight.value = value + build_El.TopShHeight.value + \
                                    build_El.RibHeight.value + build_El.BotShUpHeight.value
        if build_El.BotShUpHeight.value + value + 45.5 > build_El.HoleHeight.value:
            build_El.HoleHeight.value = build_El.BotShUpHeight.value + value + 45.5


def variation_hole(build_El, name, value):
    if name == "HoleHeight":
        if value > build_El.BeamHeight.value - build_El.TopShHeight.value - 45.5:
            build_El.HoleHeight.value = build_El.BeamHeight.value - build_El.TopShHeight.value - 45.5
        elif value < build_El.BotShLowHeight.value + build_El.BotShUpHeight.value + 45.5:
            build_El.HoleHeight.value = build_El.BotShLowHeight.value + build_El.BotShUpHeight.value + 45.5
    if name == "HoleDepth":
        if value >= build_El.BeamLength.value / 2.:
            build_El.HoleDepth.value = build_El.BeamLength.value / 2. - 45.5
