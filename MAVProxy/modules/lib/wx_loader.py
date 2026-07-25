from MAVProxy.modules.lib import wx_util
from MAVProxy.modules.lib.mp_i18n import tr

if not wx_util.safe:
    print(tr("cannot_access_wx_from_main_thread"))
    import traceback
    print(traceback.print_stack())
    raise Exception('Cannot access wx from main thread')

import wx
