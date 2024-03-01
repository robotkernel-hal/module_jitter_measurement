from conan import ConanFile


class MainProject(ConanFile):
    python_requires = "conan_template/[^5.0.6]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_jitter_measurement"
    description = "robotkernel jitter measurement module."
    exports_sources = ["*", "!.gitignore"]
    requires = ["robotkernel/[~5]@robotkernel/stable", "service_provider_process_data_inspection/[~5]@robotkernel/stable"]
