%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/jazzy/.*$
%global __requires_exclude_from ^/opt/ros/jazzy/.*$

Name:           ros-jazzy-rclpy
Version:        7.1.3
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS rclpy package

License:        Apache License 2.0
Source0:        %{name}-%{version}.tar.gz

Requires:       python%{python3_pkgversion}-yaml
Requires:       ros-jazzy-action-msgs
Requires:       ros-jazzy-ament-index-python
Requires:       ros-jazzy-builtin-interfaces
Requires:       ros-jazzy-lifecycle-msgs
Requires:       ros-jazzy-rcl
Requires:       ros-jazzy-rcl-action
Requires:       ros-jazzy-rcl-interfaces
Requires:       ros-jazzy-rcl-lifecycle
Requires:       ros-jazzy-rcl-logging-interface
Requires:       ros-jazzy-rcl-yaml-param-parser
Requires:       ros-jazzy-rmw
Requires:       ros-jazzy-rmw-implementation
Requires:       ros-jazzy-rosgraph-msgs
Requires:       ros-jazzy-rosidl-runtime-c
Requires:       ros-jazzy-rpyutils
Requires:       ros-jazzy-unique-identifier-msgs
Requires:       ros-jazzy-ros-workspace
BuildRequires:  ros-jazzy-ament-cmake
BuildRequires:  ros-jazzy-lifecycle-msgs
BuildRequires:  ros-jazzy-pybind11-vendor
BuildRequires:  ros-jazzy-python-cmake-module
BuildRequires:  ros-jazzy-rcl
BuildRequires:  ros-jazzy-rcl-action
BuildRequires:  ros-jazzy-rcl-interfaces
BuildRequires:  ros-jazzy-rcl-lifecycle
BuildRequires:  ros-jazzy-rcl-logging-interface
BuildRequires:  ros-jazzy-rcl-yaml-param-parser
BuildRequires:  ros-jazzy-rcpputils
BuildRequires:  ros-jazzy-rcutils
BuildRequires:  ros-jazzy-rmw
BuildRequires:  ros-jazzy-rmw-implementation
BuildRequires:  ros-jazzy-rmw-implementation-cmake
BuildRequires:  ros-jazzy-rosidl-runtime-c
BuildRequires:  ros-jazzy-unique-identifier-msgs
BuildRequires:  ros-jazzy-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  python%{python3_pkgversion}-pytest
BuildRequires:  ros-jazzy-ament-cmake-gtest
BuildRequires:  ros-jazzy-ament-cmake-pytest
BuildRequires:  ros-jazzy-ament-lint-auto
BuildRequires:  ros-jazzy-ament-lint-common
BuildRequires:  ros-jazzy-rosidl-generator-py
BuildRequires:  ros-jazzy-test-msgs
%endif

%description
Package containing the Python client.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/jazzy" \
    -DAMENT_PREFIX_PATH="/opt/ros/jazzy" \
    -DCMAKE_PREFIX_PATH="/opt/ros/jazzy" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/jazzy/setup.sh" ]; then . "/opt/ros/jazzy/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/jazzy

%changelog
* Wed Dec 18 2024 Shane Loretz <sloretz@openrobotics.org> - 7.1.3-1
- Autogenerated by Bloom

* Fri Sep 06 2024 Shane Loretz <sloretz@openrobotics.org> - 7.1.2-1
- Autogenerated by Bloom

* Fri Apr 19 2024 Shane Loretz <sloretz@openrobotics.org> - 7.1.1-2
- Autogenerated by Bloom

* Tue Apr 16 2024 Shane Loretz <sloretz@openrobotics.org> - 7.1.1-1
- Autogenerated by Bloom

* Thu Mar 28 2024 Shane Loretz <sloretz@openrobotics.org> - 7.1.0-1
- Autogenerated by Bloom

* Thu Mar 28 2024 Shane Loretz <sloretz@openrobotics.org> - 7.0.1-3
- Autogenerated by Bloom

* Wed Mar 06 2024 Shane Loretz <sloretz@openrobotics.org> - 7.0.1-2
- Autogenerated by Bloom

