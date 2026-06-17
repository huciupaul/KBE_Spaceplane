from math import radians, tan, asin, degrees, pi, sqrt

from numpy.ma.core import arcsin
from parapy.core import Input, Attribute, Part, child
from parapy.geom import GeomBase, LoftedShell, MirroredSurface, rotate
import kbeutils.avl as avl

# You might get a syntax error because the IDE cannot find the package 'aircraft'
# of which this file is a module. However, there is no problem with running this file
# from outside the package (i.e. from 'my_aircraft.py') because then the package can be found
# and the modules can be accessed. This is a characteristic of relative imports (as opposed to absolute imports)
# Furthermore, the reason why Fuselage and Wing can be imported from the package
# directly is thanks to the __init__.py definition. Take a look!
from section import Section


class Wing_v3(GeomBase):
    name: str = Input()
    # span: float = Input()
    # aspect_ratio: float = Input()
    # taper_ratio: float = Input()
    # le_sweep: float = Input()
    twist: float = Input()
    airfoil_name: str = Input()
    is_mirrored: bool = Input(True)

    # Control surfaces
    control_name: str = Input(None)
    control_hinge_loc: float = Input(None)
    duplicate_sign: int = Input(1)

    c_l_max: float = Input()
    ar: float = Input()
    W_S: float = Input()
    mtow: float = Input()
    design_mach: float = Input()

    @Attribute
    def wing_area(self):
        return self.mtow / self.W_S

    @Attribute  # le_sweep s.t. the wing is behind the Mach cone
    def le_sweep(self):
        return 90 - degrees(asin(1/self.design_mach))

    @Attribute
    def taper_ratio(self):
        return 0.2 * (2-self.le_sweep * pi/180)

    @Attribute
    def half_span(self):
        return sqrt(self.wing_area * self.ar) / 2

    @Attribute
    def chords(self):
        root = 2 * self.wing_area / (1+self.taper_ratio) / (2 * self.half_span)
        tip = root * self.taper_ratio
        return root, tip

    # @Attribute
    # def planform_area(self):
    #     return self.span**2 / self.aspect_ratio

    # @Attribute
    # def half_span(self):
    #     return self.span /2 if self.is_mirrored else self.span

    # @Attribute
    # def chords(self):
    #     root = ((2 * self.planform_area) /
    #             (self.span * (1 + self.taper_ratio)))
    #     tip = root * self.taper_ratio
    #     return root, tip
    #
    # @Attribute
    # def chord_root(self):
    #     return self.chords[0]

    @Attribute
    def mac(self):
        return 2/3*self.chords[0]*(1+self.taper_ratio+self.taper_ratio**2)/(1+self.taper_ratio)

    @Attribute
    def section_positions(self):
        sweep = radians(self.le_sweep)
        root = self.position
        tip = rotate(self.position.translate('x', self.half_span * tan(sweep),'y', self.half_span),
                     'y', self.twist, deg=True)
        return root, tip

    @Part
    def sections(self):
        return Section(quantify=len(self.chords),
                       airfoil_name=self.airfoil_name,
                       chord=self.chords[child.index],
                       position=self.section_positions[child.index],

                       control_name=self.control_name,
                       control_hinge_loc=self.control_hinge_loc,
                       duplicate_sign=self.duplicate_sign
                       )

    @Part
    def surface(self):
        return LoftedShell(profiles=[section.curve for section in self.sections],
                           mesh_deflection=0.0001)

    @Part
    def mirrored(self):
        return MirroredSurface(surface_in=self.surface.faces[0],
                               reference_point=self.position.point,
                               vector1=self.position.Vx,
                               vector2=self.position.Vz,
                               suppress=not self.is_mirrored,
                               mesh_deflection=0.0001)

    @Part
    def avl_surface(self):
        """Defines an AVL surface, based on the section camberlines"""
        return avl.Surface(name=self.name,
                           n_chordwise=12,
                           chord_spacing=avl.Spacing.cosine,
                           n_spanwise=20,
                           span_spacing=avl.Spacing.cosine,
                           y_duplicate=self.position.point[1] if self.is_mirrored else None,
                           sections=[section.avl_section
                                     for section in self.sections])


if __name__ == '__main__':
    from parapy.gui import display
    wng = Wing_v3(name='wing',
               # span=20,
               # aspect_ratio=6,
               # taper_ratio=0.2,
               # le_sweep=30,
               c_l_max=1.3,
               ar=2.5,
               W_S=500,
               mtow=150,
               design_mach=2,
               twist=-5,
               airfoil_name='0010',
               is_mirrored=True)
    display(wng)