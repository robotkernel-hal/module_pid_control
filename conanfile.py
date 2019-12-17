from conans import tools, python_requires

base = python_requires("conan_template/[~=5]@robotkernel/stable")

class MainProject(base.RobotkernelConanFile):
    name = "module_pid_control"
    description = "robotkernel-5 is a modular, easy configurable hardware abstraction framework"
    exports_sources = "src/*", "README.wiki", "project.properties", "module_pid_control.pc.in", "Makefile.am", "m4/*", "configure.ac", "LICENSE"
    requires = (
            "robotkernel/[~=5]@robotkernel/stable",
            "service_provider_process_data_inspection/[~=5]@robotkernel/stable" )

