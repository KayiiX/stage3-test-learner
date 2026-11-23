from setuptools import setup

setup(
    name="kfupm_test_learner",
    version="1.0.0",
    packages=["kfupm_test_learner"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/kfupm_test_learner"]),
                ("share/kfupm_test_learner", ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["controller = kfupm_test_learner.controller:main"]},
)
