from parapy.geom import GeomBase, ScaledCurve
from parapy.core import Input, Attribute, Part, DynamicType
from parapy.core.validate import AdaptedValidator

import kbeutils.avl as avl
from kbeutils.geom import Naca5AirfoilCurve, Naca4AirfoilCurve

# kbeutils.geom.curve import Naca5AirfoilCurve, Naca4AirfoilCurve

# if the lambda expression confuses you: feel free to ignore it.
_len_4_or_5 = AdaptedValidator(lambda a: 4 <= len(a) <= 5)
# If you'd still like to know what it does, the line above is equivalent to:
#
# def check_len_within_4_5(a):
#     return 4 <= len(a) <= 5     # returns True if len(a) is in the interval, otherwise False
# _len_4_or_5 = AdaptedValidator(check_len_within_4_5) # AdaptedValidator() requires a function as argument which returns True or False
#
# so by using a lambda expression, we can save some typing, but it's not necessary!


class Section(GeomBase):
    # This Input has a validator!
    # If the function returns `False` for the input, ParaPy will let you know
    # that the Input is not valid.
    airfoil_name: str = Input(validator=_len_4_or_5)
    chord: float = Input()

    control_name: str = Input(None)
    control_hinge_loc: float = Input(None)
    duplicate_sign: int = Input(1)

    @Attribute
    def avl_controls(self):
        """configure what (if anything) AVL can modify about this section"""
        if self.control_name is not None and self.control_hinge_loc is not None:
            return [avl.Control(name=self.control_name,
                                gain=1.0,
                                x_hinge=self.control_hinge_loc,
                                duplicate_sign=self.duplicate_sign)]
        else:
            return []

    @Part
    def airfoil(self):
        return DynamicType(type=Naca5AirfoilCurve if len(self.airfoil_name) == 5
                                                     else Naca4AirfoilCurve,
                           designation=self.airfoil_name,
                           mesh_deflection=0.00001,
                           hidden=True,
                           # cst_order_upper=3, # Changing the order of the CST coefficients for each surface
                           # cst_order_lower=2, # of the airfoil. This is just an example, and they are usually the
                                                # same for both upper and lower surfaces.
                           )

    @Attribute
    def cst_coeffs_upper(self):
        """Upper CST Coefficients of the airfoil. These can be useful in applications like Q3D, which
        require CST coefficients to analyze wings."""
        return self.airfoil.cst_upper

    @Attribute
    def cst_coeffs_lower(self):
        """Lower CST Coefficients of the airfoil"""
        return self.airfoil.cst_lower


    @Part
    def curve(self):
        """The actual airfoil curve"""
        return ScaledCurve(self.airfoil,
                           self.position.point,
                           self.chord,
                           mesh_deflection=0.00001)

    @Part
    def avl_section(self):
        """Defines the section for AVL (zero-thickness profile, representing
        the camber of the airfoil. Any input curve is allowed. It includes
        avl_section_by_points """
        return avl.SectionFromCurve(curve_in=self.curve,
                                    controls=self.avl_controls)

    # Other variations of this method, which represent the airfoil differently:

    # @Part
    # def avl_section_no_curvature(self):
    #    """This version ignores the camber of the airfoil"""
    #     return avl.Section(chord=self.chord)

    # @Part
    # def avl_section(self):
    #     """the camber of the airfoil is accounted, but works only for NACA4
    #     airfoils (AVL limitation)"""
    #     return avl.Section(chord=self.chord,
    #                        airfoil=avl.NacaAirfoil(designation=self.airfoil_name))


    # @Part
    # def avl_section(self):
    #     """Camberline based on point coordinates. Camber is accounted for,
    #     any curve is allowed."""
    #     return avl.Section(chord=self.chord,
    #                        airfoil=avl.DataAirfoil(self.curve.sample_points))




if __name__ == '__main__':
    from parapy.gui import display

    sec = Section(airfoil_name="2412",
                  chord=2,
                  label="wing section",
                  )
    display(sec)