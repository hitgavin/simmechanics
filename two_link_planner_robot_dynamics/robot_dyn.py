import sympy
import sympybotics

l1 = 0.3
l2 = 0.2

rbtdef = sympybotics.RobotDef("two_link_planner", 
        [('pi/2', 0, 0, 'q'),(0, l1, 0, 'q')],
        dh_convention='modified')

rbtdef.frictionmodel = {'Coulomb', 'viscous'}
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81])

print('dynamic parameters')
print(rbtdef.dynparms())

rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)

rbt.calc_base_parms()
print('minimum inertial parameters')
rbt.dyn.baseparms
print(rbt.dyn.baseparms)
print('observation matrix')
rbt.Hb_code
print(rbt.Hb_code)

