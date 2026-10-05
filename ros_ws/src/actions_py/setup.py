from setuptools import find_packages, setup

package_name = "actions_py"

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
    description="Python servers and clients for the SONHO action exercises.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "action_servers = actions_py.action_servers:main",
            "action_client = actions_py.action_client:main",
        ],
    },
)
