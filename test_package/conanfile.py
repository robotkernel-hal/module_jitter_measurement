import os
import shutil

from conan import ConanFile
from conan.tools.build import can_run


class TestTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    exports_sources = "mod_test.rkc"
    generators = "VirtualRunEnv"

    def test(self):
        rkc_path = os.path.join(self.source_folder, "mod_test.rkc")
        shutil.copy(rkc_path, "mod_test.rkc")

        if can_run(self):
            self.run("robotkernel --test-run --config .%smod_test.rkc" % os.sep, env="conanrun")
        else:
            self.output.warn("Skipping run cross built package")
