from conans import tools, python_requires

base = python_requires("conan_template/[~=5]@robotkernel/stable")

class MainProject(base.RobotkernelConanFile):
    name = "module_jitter_measurement"
    description = "robotkernel-5 jitter measurement module."
    exports_sources = ["*", "!.gitignore"] + ["!%s" % x for x in tools.Git().excluded_files()]

    def requirements(self):
        self.requires("robotkernel/[~=5]@robotkernel/stable")
        self.requires("service_provider_process_data_inspection/[~=5]@robotkernel/stable")

