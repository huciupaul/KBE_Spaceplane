
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

from parapy.geom import FittedCurve, Point
from parapy.core import Attribute, Part, Input
from ref_frame import Frame


class Airfoil(FittedCurve):
    # note the use of FittedCurve as superclass
    # This is makes the code more compact but means that we must ensure:
    # - that all required Inputs for the FittedCurve class are provided
    # - that we don't re-define any of its internal methods or slots in a
    #   way that causes trouble.

    # The alternative is to inherit from (parapy.geom.)GeomBase and include a
    # @Part which is a Fitted curve. This makes the object tree in Parapy more
    # complex (...`lifttingsurface.airfoil.airfoilcurve` instead of
    # `liftingsurface.airfoil`) and potentially confusing, but avoids the risk
    # Recommendation: Always clearly mark definitions required by the superclass
    # clearly with a comment. Also, Pycharm shows inherited methods and slots
    # in the code structure graph (Alt+7, or the icon of three block in the shape of an L),
    # so you can check what your class is inheriting or redefining.


    chord: float = Input(1.)
    airfoil_name: str = Input("whitcomb")
    thickness_factor: float = Input(1.)
    mesh_deflection: float = Input(1e-4)

    # - required slot for the FittedCurve superclass -
    # By adding the file parsing code, it can compute the list of points
    # or read them from a file, instead of expecting them as input.
    #! This still works with an @Input decorator! In that case, passing the points directly is possible,
    #! but it will parse them from the airfoil file otherwise (try it out!)
    @Attribute
    def points(self) -> [Point]:
        """List of points defining the airfoil shape, read from a file"""
        with open(self.airfoil_name + ".dat", 'r') as f:
            point_lst = []
            for line in f:
                x, z = line.split(' ', 1)  # the cartesian coordinates are directly interpreted as X and Z coordinates
                point_lst.append(self.position.translate(
                    x=float(x) * self.chord,  # the x points are scaled according to the airfoil chord length
                    z=float(z) * self.chord * self.thickness_factor).location) # the y points are scaled according to the /
                                                                         # thickness factor
        return point_lst

    @Part
    def airfoil_frame(self):  # to visualize the given airfoil reference frame
        return Frame(pos=self.position,
                     hidden=False)


if __name__ == '__main__':
    from parapy.gui import display
    foil = Airfoil(label="airfoil", airfoil_name='whitcomb')
    display(foil)
