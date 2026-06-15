from setuptools import find_packages, setup

package_name = "navigation_module"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "examples", "resource", "test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={
        "dev": ["pytest>=7.4.0"],
    },
    zip_safe=True,
    description="Библиотека 2D-навигации GPS+IMU (чистый Python).",
)
