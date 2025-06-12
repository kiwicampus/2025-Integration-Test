from setuptools import find_packages, setup

package_name = "rclpy_tests"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wilmer Garzon",
    maintainer_email="wilmer.garzon@kiwibot.com",
    description="Rclpy unit testing framework",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
