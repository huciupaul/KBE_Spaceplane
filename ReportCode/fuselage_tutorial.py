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

from parapy.geom import (LoftedSolid, LoftedSurface, Circle, Vector, translate,
                        Rectangle, ProjectedCurve, rotate90)

from parapy.core import Input, Attribute, Part, child


class Fuselage(LoftedSolid):
    """Fuselage geometry, a loft through circles.

    Note the use of LoftedSolid as superclass. It means that every
    Fuselage instance defines a lofted geometry. A required input for LoftedSolid
    is a list of profiles, so either an @Attribute or a @Part sequence
    called "profiles" must be present in the body of the class. Use 'Display
    node' in the root node of the object tree to visualise the (yellow) loft
    in the GUI graphical viewer

    Examples:
        >>> obj = Fuselage(pass_down="fu_radius, fu_sections, fu_length",
        ...                    color="Green",
        ...                    mesh_deflection=0.0001
        ...                    )
    """

    #: fuselage radius
    radius: float = Input()
    #: fuselage sections (percentage of nominal radius, at evenly-spaced stations)
    sections: list[float] = Input([10, 90, 100, 100, 100, 100, 100, 100, 95, 70, 5])
    #: fuselage length (m)
    length: float = Input()

    ex_pos_rel: float = Input(doc='relative longitudinal position of exists')

    # Exit size has defaults because there are standardized dimensions from
    # ICAO/FAR standards (actual values below not accurate, of course)
    ex_height: float = Input(1.7,
                             doc='exit height. Default: CS-25 type IX (1.7m)')
    ex_width: float = Input(0.7,
                            doc='exit width. default: CS-25 type IX (0.7cm)')

    mesh_deflection: float = Input(1e-4)

    @Attribute
    def section_radius(self) -> list[float]:
        """Section radius multiplied by the radius distribution
        through the length. Note that the numbers are percentages.

        """
        return [i * self.radius / 100. for i in self.sections]

    @Attribute
    def section_length(self) -> float:
        """Section length is determined by dividing the fuselage
        length by the number of fuselage sections.
        """
        return self.length / (len(self.sections) - 1)

    # Required slot of the superclass LoftedSolid.
    # Originally, this is an Input slot, but any slot type is fine as long as it contains
    # an iterable of the profiles for the loft.
    @Part
    def profiles(self):
        return Circle(quantify=len(self.sections), color="Black",
                      radius=self.section_radius[child.index],
                      # fuselage along the X axis, nose in XOY
                      position=translate(self.position.rotate90('y'),  # circles are in XY plane, thus need rotation
                                         Vector(1, 0, 0),
                                         child.index * self.section_length),
                      hidden=True)


    @Part
    def owe_planar(self):
        '''planar over-wing exit geometry, for projection onto surface'''
        return Rectangle(width=self.ex_width,
                         length=self.ex_height,
                         # rotate by 90° to align `length` with z axis
                         # move to axial position and offset from symmetry to
                         # avoid non-unique result
                         position=translate(rotate90(self.position, 'x'),
                                            x=self.ex_pos_rel * self.length,
                                            z=self.radius),
                         hidden=True)

    @Part(in_tree=False)
    def owe_raw(self):
        """
        Over-wing exit geometry, defined by projecting the `rectangle` part
        onto fuselage surface. Only shows one out of the two projections but
        evaluates and stores both of them, as self.wires. This is referenced by
        `overwing_exits` for display.
        """
        return ProjectedCurve(source=self.owe_planar,
                              target=self,
                              direction=self.position.Vy,
                              hidden=True)

    @Attribute(in_tree=True)
    def overwing_exits(self):
        """Shows both over-wing exists, computed by `over_wing_exit` @Part"""
        return self.owe_raw.wires


if __name__ == '__main__':
    from parapy.gui import display
    fus = Fuselage(label="fuselage", radius=1.5, length=15,
                   ex_pos_rel=0.5,
                   mesh_deflection=1e-4)
    display(fus)
