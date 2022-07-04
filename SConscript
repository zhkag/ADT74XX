from building import *

cwd     = GetCurrentDir()
src     = Glob('*.c')
path    = [cwd]

group = DefineGroup('adt74xx', src, depend = ['PKG_USING_ADT74XX'], CPPPATH = path)

Return('group')