from conans import ConanFile, tools
import os

class MainProject(ConanFile):
    python_requires = "conan_template_ln_generator/[~=5 >=5.0.7]@robotkernel/stable"
    python_requires_extend = "conan_template_ln_generator.RobotkernelLNGeneratorConanFile"

    name = "module_jitter_measurement"
    description = "robotkernel jitter measurement module."
    exports_sources = ["*", "!.gitignore"] + ["!%s" % x for x in tools.Git().excluded_files()]
    requires = [    
        "robotkernel/[~=5]@robotkernel/stable", 
        "service_provider_process_data_inspection/[~=5]@robotkernel/stable" ]

