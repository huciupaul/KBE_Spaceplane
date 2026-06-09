# -*- coding: utf-8 -*-

from math import radians

from parapy.core import Input, Part, Attribute
from parapy.geom import (GeomBase, MirroredShape, translate, rotate,)

from liftingsurface import LiftingSurface
from ref_frame import Frame

class Wing(GeomBase):

    airfoil_root_name: str = Input("whitcomb")
    airfoil_kink_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("whitcomb")
    c_root: float = Input()
    c_kink : float = Input()
    c_tip: float = Input()

    #: to reduce/increase the thickness of the airfoil from the .dat file
    t_factor_root: float = Input(1)
    t_factor_kink: float = Input(1)
    t_factor_tip: float = Input(1)
    # Default: Uses airfoils as provided, no change to thickness

    semi_span: float = Input()
    kink_span: float = Input()
    sweep_kink: float = Input(0)  # at leading edge, in degrees
    sweep_tip: float = Input(0)

    twist: float = Input(0)  # tip airfoil twist angle, measured around leading edge, in degrees

    dihedral: float = Input(0)

    # default: fairly coarse to avoid slowing down things
    mesh_deflection: float = Input(1e-3)

    @Part
    def frame(self):
        """This helps visualise the wing local reference frame"""
        return Frame(pos=self.position)


    # Note: realistically, the wing should be *sheared*, not rotated to achieve
    # dihedral.That is: it should work in the same way as sweep!

    @Part
    def starboard_geo(self):
        return LiftingSurface(pass_down="airfoil_root_name, airfoil_kink_name, airfoil_tip_name,"
                                        "t_factor_root, t_factor_root, t_factor_tip,"
                                        "sweep_kink, sweep_tip, twist, mesh_deflection",
                              c_root=self.c_root,
                              c_kink=self.c_kink,
                              c_tip=self.c_tip,
                              semi_span=self.semi_span,
                              kink_span=self.kink_span,
                              position=rotate(self.position,
                                              'x', radians(self.dihedral)),
                              mesh_deflection=self.mesh_deflection,
                              ruled=True)

    @Part
    def port_geo(self):
        return MirroredShape(shape_in=self.starboard_geo,
                             reference_point=self.position,
                             # Two vectors and a point to define the mirror plane
                             vector1=self.position.Vz,
                             vector2=self.position.Vx,
                             mesh_deflection=self.mesh_deflection)

    @Attribute
    def solids(self):
        """List of all solid bodies of the wing, for use in boolean operations"""
        # note: boolean merge of these may not work if they barely touch (or not)
        return [self.starboard_geo, self.port_geo]

    @Attribute
    def cog(self):
        # not ideal: we can't compute cog by weighted averaging of all solids
        # (with volume as weight), because the port wing, as a
        # `MirrorredShape`, has no `volume` attribute
        return self.starboard_geo.cog.project(self.position,
                                              self.position.Vx,
                                              self.position.Vz)





if __name__ == '__main__':
    from parapy.gui import display
    testwing = Wing(semi_span=19,
                    kink_span=9,
                    sweep_kink=15,
                    sweep_tip=20,
                    twist=0,
                    dihedral=4,
                    c_root=6,
                    c_kink=5,
                    c_tip=2.3,
                    t_factor_root=1,
                    t_factor_kink=0.75,
                    t_factor_tip=0.5,
                    airfoil_root_name='whitcomb',
                    airfoil_kink_name='whitcomb',
                    airfoil_tip_name='whitcomb',
                    mesh_deflection=1e-5)
    display(testwing)
