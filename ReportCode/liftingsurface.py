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

from math import radians, tan
from parapy.geom import LoftedSolid, translate, rotate
from parapy.core import Input, Attribute, Part
from airfoil import Airfoil  # note this is a different class than n exe_17_wing.py
from ref_frame import Frame


class LiftingSurface(LoftedSolid):  # note use of loftedSolid as superclass
    airfoil_root_name: str = Input("whitcomb")
    airfoil_kink_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("whitcomb")

    c_root: float = Input()
    c_kink: float = Input()
    c_tip: float = Input()
    t_factor_root: float = Input(1.)
    t_factor_kink: float = Input(1.)
    t_factor_tip: float = Input(1.)


    semi_span: float = Input()
    kink_span: float = Input()
    sweep_kink: float = Input(0)
    sweep_tip: float = Input(0)

    twist: float = Input(0)

    mesh_deflection: float = Input(1e-4)

    ruled: bool = Input(True)

    # required slot for the superclass LoftedSolid
    # (usually an @Input, but we're turning it into an @Attribute)
    @Attribute
    def profiles(self):
        return [self.root_airfoil,self.kink_airfoil, self.tip_airfoil]
    # Because the class inherits from `LoftedSolid`, it requires profiles to loft over, and expects
    # to find them as a slot called `profiles`. When instantiating a `LoftedSolid` object, the profiles would  normally,
    # be an `Input` slot, but we can also serve them as an `Attribute` -- as long as `self.profiles` is present.
    # try commenting this part out and see how it fails!

    @Part(in_tree=False)
    def lifting_surf_frame(self):
        """to visualize the given lifting surface reference frame"""
        return Frame(pos=self.position, hidden=True)

    @Part
    def root_airfoil(self):  # root airfoil will receive self.position as default
        return Airfoil(airfoil_name=self.airfoil_root_name,
                       chord=self.c_root,
                       thickness_factor=self.t_factor_root,
                       mesh_deflection=self.mesh_deflection,
                       hidden=False)

    @Part
    def kink_airfoil(self):
        return Airfoil(airfoil_name=self.airfoil_kink_name,
                       chord=self.c_kink,
                       thickness_factor=self.t_factor_kink,
                       position=translate(
                           rotate(self.position, "y", radians(self.twist)),  # apply twist angle
                           "y", self.kink_span,
                           "x", self.kink_span * tan(radians(self.sweep_kink))),
                       mesh_deflection=self.mesh_deflection,
                       hidden=False)

    @Part
    def tip_airfoil(self):
        return Airfoil(airfoil_name=self.airfoil_tip_name,
                       chord=self.c_tip,
                       thickness_factor=self.t_factor_tip,
                       position=translate(
                           rotate(self.position, "y", radians(self.twist)),  # apply twist angle
                           "y", self.semi_span,
                           "x", self.kink_span * tan(radians(self.sweep_kink)) + (self.semi_span-self.kink_span) * tan(radians(self.sweep_tip))),  # apply sweep
                       mesh_deflection=self.mesh_deflection,
                       hidden=False)


if __name__ == '__main__':
    from parapy.gui import display
    ls = LiftingSurface(label="lifting surface",
                        c_root=5,
                        c_kink=5,
                        c_tip=2.5,
                        semi_span=27,
                        kink_span=10,
                        mesh_deflection=1e-4)
    display(ls)
