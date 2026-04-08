from conan import ConanFile

class MainProject(ConanFile):
    python_requires = "conan_template/[^6]@robotkernel/unstable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_jitter_measurement"
    description = "robotkernel jitter measurement module."
    exports_sources = ["*", "!.gitignore"]
    tool_requires = ["robotkernel_generator/[~6]@robotkernel/unstable"]

    def requirements(self):
        self.requires(f"{self.name}_ln_msgdef/{self.version}@{self.user}/{self.channel}")
        self.requires("service_provider_process_data_inspection/[~6]@robotkernel/unstable")
        self.requires("robotkernel/[~6]@robotkernel/unstable")

