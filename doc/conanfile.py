from conan import ConanFile
from conan import tools
from conan.tools.files import mkdir, chdir, copy
from conan.tools.gnu import Autotools, AutotoolsToolchain
from conan.tools.scm import Git
import os

class MainProject(ConanFile):
    name = "module_jitter_measurement_doc"
    license = "GPLv3"
    author = "Robert Burger <robert.burger@dlr.de>"
    url = f"https://rmc-github.robotic.dlr.de/robotkernel/module_jitter_measurement.git"
    description = "robotkernel jitter measurement module."
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = ["*", "!.gitignore", "!bindings"]
    options = {"shared": [True, False],
               "coverage" : [True, False]}
    default_options = {"shared": True,
                       "coverage" : False}

    build_requires = [
        "sphinx-rtd-theme/[~2]@pypi/stable",
        "sphinxcontrib-jquery/[>=4 <5]@pypi/stable",
    ]

    def generate(self):
        tc = AutotoolsToolchain(self)
        tc.generate()

    def build(self):
        autotools = Autotools(self)
        autotools.make(target="html")

    def package(self):
        git = Git(self, folder=self.recipe_folder)
        remoteurl = git.get_remote_url()

        autotools = Autotools(self)
        autotools.install(args=[f'REMOTEURL={remoteurl}'])

    def package_info(self):
        self.cpp_info.includedirs = ['include']
        self.cpp_info.libs = ["osal"]
        self.cpp_info.bindirs = ['bin']
        self.cpp_info.resdirs = ['share']
