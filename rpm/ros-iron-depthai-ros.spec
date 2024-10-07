%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/iron/.*$
%global __requires_exclude_from ^/opt/ros/iron/.*$

Name:           ros-iron-depthai-ros
Version:        2.10.2
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS depthai-ros package

License:        MIT
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-iron-depthai
Requires:       ros-iron-depthai-bridge
Requires:       ros-iron-depthai-descriptions
Requires:       ros-iron-depthai-examples
Requires:       ros-iron-depthai-filters
Requires:       ros-iron-depthai-ros-driver
Requires:       ros-iron-depthai-ros-msgs
Requires:       ros-iron-ros-workspace
BuildRequires:  ros-iron-ament-cmake
BuildRequires:  ros-iron-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
The depthai-ros package

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/iron" \
    -DAMENT_PREFIX_PATH="/opt/ros/iron" \
    -DCMAKE_PREFIX_PATH="/opt/ros/iron" \
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
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/iron

%changelog
* Mon Oct 07 2024 Adam Serafin <adam.serafin@luxonis.com> - 2.10.2-1
- Autogenerated by Bloom

* Fri Sep 20 2024 Adam Serafin <adam.serafin@luxonis.com> - 2.10.1-1
- Autogenerated by Bloom

* Thu Aug 29 2024 Adam Serafin <adam.serafin@luxonis.com> - 2.10.0-1
- Autogenerated by Bloom

* Thu Jan 25 2024 sachin <sachin@luxonis.com> - 2.9.0-1
- Autogenerated by Bloom

* Tue Oct 17 2023 sachin <sachin@luxonis.com> - 2.8.2-1
- Autogenerated by Bloom

* Fri Sep 15 2023 sachin <sachin@luxonis.com> - 2.8.1-1
- Autogenerated by Bloom

* Thu Sep 07 2023 sachin <sachin@luxonis.com> - 2.8.0-1
- Autogenerated by Bloom

* Fri Aug 11 2023 sachin <sachin@luxonis.com> - 2.7.5-1
- Autogenerated by Bloom

* Mon Jun 26 2023 sachin <sachin@luxonis.com> - 2.7.4-1
- Autogenerated by Bloom
