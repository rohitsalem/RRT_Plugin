    #!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/rrt_plugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    rrt_module = RaveCreateModule(env,'rrt_module')
    print rrt_module.SendCommand('help')
finally:
    RaveDestroy()
