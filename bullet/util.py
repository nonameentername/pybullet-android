# -*- coding: utf-8 -*-
"""
    :copyright: 2009 by Florian Boesch <pyalot@gmail.com>.
    :license: GNU AGPL3, see LICENSE for more details.
"""

import os

def resource(module, *path):
    here = os.path.dirname(
        os.path.abspath(module)
    )
    return os.path.join(here, *path)
