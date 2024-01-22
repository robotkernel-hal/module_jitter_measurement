import os
import shutil

from conans import CMake, ConanFile, tools
from conans.client.run_environment import RunEnvironment


class TestTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "mod_test.rkc"

    def test(self):
        rkc_path = os.path.join(self.source_folder, "mod_test.rkc")
        shutil.copy(rkc_path, "mod_test.rkc")

        if not tools.cross_building(self.settings):
            re = RunEnvironment(self)
            with tools.environment_append(re.vars):
                self.run("robotkernel --test-run --config .%smod_test.rkc" % os.sep)
        else:
            self.output.warn("Skipping run cross built package")
