# -*- coding: utf-8 -*-
"""
    :copyright: 2008 by Florian Boesch <pyalot@gmail.com>.
    :license: GNU AGPL, see LICENSE for more details.
"""
    
from setuptools import setup

setup(
    name                    = 'bullet',
    version                 = '0.1.0', 
    description             = 'constraint based terrain UV mapping',
    long_description        = __doc__,
    license                 = 'GNU AFFERO GENERAL PUBLIC LICENSE (AGPL) Version 3',
    url                     = 'http://hg.codeflow.org/bullet',
    download_url            = 'http://hg.codeflow.org/bullet/archive/tip.tar.gz',
    author                  = 'Florian Boesch',
    author_email            = 'pyalot@gmail.com',
    maintainer              = 'Florian Boesch',
    maintainer_email        = 'pyalot@gmail.com',
    zip_safe                = False,
    include_package_data    = True,
    packages                = ['bullet'],
    package_data            = {
        'bullet' : ['bin/*.so'],
    },
    install_requires        = [],
    platforms               = ['any'],
)
