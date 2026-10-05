from setuptools import find_packages, setup

package_name = "services_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samuel Tirello",
    maintainer_email="samutirello@gmail.com",
    description="Python servers and clients for the ten SONHO service exercises.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "service_servers = services_py.service_servers:main",
            "service_client = services_py.service_client:main",
        ],
    },
)
