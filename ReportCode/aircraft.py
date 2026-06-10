#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 ParaPy Holding B.V.
#
# This file is subject to the terms and conditions defined in
# the license agreement that you have received with this source code
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
# PURPOSE.

import os
from math import radians
from parapy.exchange import STEPWriter
from parapy.geom import (GeomBase, translate, rotate, ProjectedCurve,
                         MirroredShape, Rectangle, SubtractedSolid, Subtracted,
                         Fused, FusedSolid, rotate90)
from parapy.core import Input, Attribute, Part, child, action
from liftingsurface import LiftingSurface  # note this is not the same as wing class defined in tutorial 5
from fuselage_tutorial import Fuselage
from wing import Wing
from ref_frame import Frame
from tail import TailSection


maindir = os.path.dirname(__file__)


class Aircraft(GeomBase):
    # Consider: Which of these defaults make sense, and which are not helping?
    fu_radius: float = Input(1)
    fu_sections: list[float] = Input([10, 90, 100, 100, 100, 100, 100, 100, 95, 70, 5])
    fu_length: float = Input(1)

    # airfoils default to pre-defined Whitcomb
    airfoil_root_name: str = Input("whitcomb")
    airfoil_kink_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("whitcomb")
    w_c_root: float = Input()
    w_c_kink: float = Input()
    w_c_tip: float = Input()

    #: to reduce/increase the thickness of the airfoil from the .dat file
    w_t_factor_root: float = Input(1)
    w_t_factor_kink: float = Input(1)
    w_t_factor_tip: float = Input(1)
    # Default: Uses airfoils as provided, no change to thickness

    w_semi_span: float = Input()
    w_kink_span: float = Input()
    w_sweep_kink: float = Input(0)  # at leading edge, in degrees
    w_sweep_tip: float = Input(0)
    w_twist: float = Input(0)  # tip airfoil twist angle, measured around leading edge, in degrees

    w_dihedral: float = Input(0)

    #: longitudinal position w.r.t. fuselage length. (% of fus length)
    w_pos_rel_x: float = Input(0.4)

    #: vertical position w.r.t. to fus  (% of fus radius)
    w_pos_rel_z: float = Input(0.8)

    #: longitudinal position of the vertical tail, as % of fus length
    vt_long: float = Input(0.8)
    vt_taper: float = Input(0.4)
    vt_chord_perc: float = Input(0.7)
    #? Wait a second: How are the vertical tail root and tip chord length determined?

    mesh_deflection: float = Input(1e-4)

    # whether or not to show merged solid (slow to display)
    show_merged: bool = Input()


    @Part
    def aircraft_frame(self):
        """This helps visualise the wing local reference frame"""
        return Frame(pos=self.position)

    @Part
    def fuselage(self):
        return Fuselage(radius=self.fu_radius,
                        sections=self.fu_sections,
                        length=self.fu_length,
                        color="Green",
                        mesh_deflection=self.mesh_deflection,
                        # over-wing exit is always at root mid-chord location
                        ex_pos_rel=self.w_pos_rel_x \
                                     + self.w_c_root / 2 / self.fu_length)


    @Part
    def wing(self):
        return Wing(pass_down=['airfoil_root_name', 'airfoil_kink_name' 'airfoil_tip_name'],
                    semi_span=self.w_semi_span,
                    kink_span=self.w_kink_span,
                    sweep_kink=self.w_sweep_kink,
                    sweep_tip=self.w_sweep_tip,
                    twist=self.w_twist,
                    dihedral=self.w_dihedral,
                    c_root=self.w_c_root,
                    c_kink=self.w_c_kink,
                    c_tip=self.w_c_tip,
                    t_factor_root=self.w_t_factor_root,
                    t_factor_kink=self.w_t_factor_kink,
                    t_factor_tip=self.w_t_factor_tip,
                    position=translate(self.position,
                                       'x', self.w_pos_rel_x * self.fu_length,
                                       'z', self.w_pos_rel_z * -self.fu_radius),
                    mesh_deflection=self.mesh_deflection)



    @Part
    def vert_tail(self):
        return TailSection(c_root=self.w_c_root * self.vt_chord_perc,
                              c_tip=self.w_c_root * self.vt_taper,
                              airfoil_root_name="whitcomb",
                              airfoil_tip_name="whitcomb",
                              t_factor_root=0.9 * self.w_t_factor_root,
                              t_factor_tip=0.8 * self.w_t_factor_tip,
                              semi_span=self.w_semi_span / 3,
                              sweep=45,
                              twist=0,
                              position=rotate(translate(self.position,
                                                        "x", self.vt_long * self.fu_length,
                                                        "z", self.fu_radius * 0.7),
                                              "x", radians(90)),
                              mesh_deflection=self.mesh_deflection)

    @Part
    def h_tail_right(self):
        return TailSection(c_root=self.w_c_root / 1.5,
                              c_tip=self.w_c_tip / 2,
                              airfoil_root_name="whitcomb",
                              airfoil_tip_name="whitcomb",
                              t_factor_root=0.9 * self.w_t_factor_root,
                              t_factor_tip=0.7 * child.t_factor_root,
                              semi_span=self.w_semi_span / 2.5,
                              sweep=self.w_sweep_tip * 1.2,
                              twist=0,
                              position=rotate(translate
                                              (self.position, "x", self.fu_length - self.w_c_root),
                                              "x", radians(self.w_dihedral + 4)),
                              mesh_deflection=self.mesh_deflection)

    @Part
    def h_tail_left(self):
        return MirroredShape(shape_in=self.h_tail_right,
                             reference_point=self.position,
                             # Two vectors and a point to define the mirror plane
                             vector1=self.position.Vz,
                             vector2=self.position.Vx,
                             mesh_deflection=self.mesh_deflection)


    @Part
    def aircraft_solid(self):
        return Fused(shape_in=self.fuselage,
                          tool=[*self.wing.solids,
                                self.vert_tail,
                                self.h_tail_left,
                                self.h_tail_right],
                          mesh_deflection=self.mesh_deflection,
                          color="white",
                          suppress=not(self.show_merged))

    @Part
    def step_writer_components(self):
        """Exports the components as separate parts, can be triggered through GUI or via its .write() method"""
        return STEPWriter(default_directory=maindir,
                          nodes=[self.vert_tail,
                                 self.h_tail_left,
                                 self.h_tail_right,
                                 *self.wing.solids,
                                 self.fuselage,
                                 *self.fuselage.overwing_exits])

    @Part
    def step_writer_fused(self):
        """Exports the aircraft as single part, can be triggered through GUI or via its .write() method"""
        return STEPWriter(default_directory=maindir,
                          nodes=[self.aircraft_solid],
                          suppress=not(self.show_merged))


    @action(button_label='adjust sweep')
    def adjust_sweep(self):
        """Adjust sweep.
        Update aircraft definition until wing CG matches fuselage CG,
        returns new sweep angle and number of iterations"""
        import math
        n_iter = 0
        cog_tol = 1e-6   # TODO: Make tolerance an Input/Attribute!
        cog_fuse = self.fuselage.cog
        # using starboard wing instead of joint CG to get spanwise coordinate, too
        cog_wing = self.wing.starboard_geo.cog
        # positive err means that wing CG is downstream of fuselage CG
        err = cog_wing.x - cog_fuse.x
        print(f'initial error: {err}')
        while abs(err) > cog_tol:
            # Approximation: wing CG at constant y, x proportional to y*tan(sweep)
            # tan(new) = tan(current) + err/y_cg
            cursweep = math.radians(self.w_sweep)
            newsweep = math.atan(math.tan(cursweep) - err / cog_wing.y)
            self.w_sweep = math.degrees(newsweep)
            cog_wing = self.wing.starboard_geo.cog
            err = cog_wing.x - cog_fuse.x
            n_iter += 1
            print(n_iter, err)
        return [newsweep, n_iter]


if __name__ == '__main__':
    from parapy.gui import display
    import parapy.geom as ppg
    ac = Aircraft(label="aircraft",
                   fu_radius=2.5,
                   #fu_sections=[10, 90, 100, 100, 100, 100, 100, 100, 95, 70, 10],
                   fu_length=50.65,
                   airfoil_root_name="whitcomb",
                  airfoil_kink_name="whitcomb",
                   airfoil_tip_name="simm_airfoil",
                   w_c_root=6., w_c_kink=4, w_c_tip=2.3,
                   w_t_factor_root=1, w_t_factor_kink=0.75, w_t_factor_tip=0.5,
                   w_semi_span=27., w_kink_span=17,
                   w_sweep_kink=10, w_sweep_tip=25, w_twist=-4, w_dihedral=3,
                   w_pos_rel_x=0.4, w_pos_rel_z=0.8,
                   vt_long=0.8, vt_taper=0.4,
                   # rotated to test robustness (no references to absolute coords)
                   position=ppg.XOY.rotate(x=0.7).translate(x=5, y=10),
                   # leave off for faster display
                   show_merged=False)
    display(ac)
