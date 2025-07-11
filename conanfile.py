from conan import ConanFile


class MainProject(ConanFile):
    python_requires = "conan_template/[^5.0.6]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_jitter_measurement"
    description = "robotkernel jitter measurement module."
    exports_sources = ["*", "!.gitignore"]
    tool_requires = ["robotkernel_service_helper/[~6]@robotkernel/snapshot"]
    requires = ["robotkernel/6.0.0-no-string-util@robotkernel/snapshot", "service_provider_process_data_inspection/[~6]@robotkernel/snapshot"]
