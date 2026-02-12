import os

from conan import ConanFile, conan_version
from conan.tools.files import chdir, copy
from conan.tools.scm import Version


class MainProject(ConanFile):
    name = "module_jitter_measurement_ln_msgdef"
    url = "https://rmc-github.robotic.dlr.de/robotkernel/module_jitter_measurement"
    author = "Robert Burger robert.burger@dlr.de"
    license = "GPLv3"
    description = "module_jitter_measurement ln message definitions"
    settings = None
    exports_sources = [
        "*",
    ]

    tool_requires = ["robotkernel_ln_helper/[~6]@robotkernel/unstable"]
    generators = "VirtualBuildEnv"

    def package(self):
        ln_msg_dir = "share/ln/message_definitions"

        svc_def_files = []
        with chdir(self, self.build_folder):
            for dirpath, dirnames, filenames in os.walk("module_jitter_measurement"):
                svc_def_files.extend(os.path.join(dirpath, filename) for filename in filenames)

        self.run(
            "service_generate --indir . --outdir %s %s" % (ln_msg_dir, " ".join(svc_def_files)), env="conanbuild"
        )

        copy(self, "share/ln/message_definitions/*", self.build_folder, self.package_folder)

    def package_info(self):
        msgdef_dir = os.path.join(self.package_folder, "share/ln/message_definitions")
        if Version(conan_version) < "2.0.0":
            self.env_info.LN_MESSAGE_DEFINITION_DIRS.append(msgdef_dir)
        self.runenv_info.append_path("LN_MESSAGE_DEFINITION_DIRS", msgdef_dir)
        self.buildenv_info.append_path("LN_MESSAGE_DEFINITION_DIRS", msgdef_dir)

        # This package is not providing any includes
        self.cpp_info.includedirs = []

