from conan import ConanFile


class MainProject(ConanFile):
    python_requires = "conan_template/[^5.0.6]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_jitter_measurement"
    description = "robotkernel jitter measurement module."
    exports_sources = ["*", "!.gitignore"]
    tool_requires = ["robotkernel_service_helper/6.0.0-yaml-service@robotkernel/unstable"]

    def source(self):
        self.run(f"sed 's/AC_INIT(.*/AC_INIT([robotkernel], [{self.version}], [{self.author}])/' configure.ac.in > configure.ac")

    def requirements(self):
        self.requires(f"{self.name}_ln_msgdef/{self.version}@{self.user}/{self.channel}")
        self.requires("service_provider_process_data_inspection/6.0.0-yaml-service@robotkernel/unstable")
        self.requires("robotkernel/6.0.0-yaml-service@robotkernel/unstable")

