%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

Name:           ros-humble-depthai-examples
Version:        2.10.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS depthai_examples package

License:        MIT
Source0:        %{name}-%{version}.tar.gz

Requires:       opencv-devel
Requires:       ros-humble-camera-info-manager
Requires:       ros-humble-cv-bridge
Requires:       ros-humble-depth-image-proc
Requires:       ros-humble-depthai
Requires:       ros-humble-depthai-bridge
Requires:       ros-humble-depthai-descriptions
Requires:       ros-humble-depthai-ros-msgs
Requires:       ros-humble-foxglove-msgs
Requires:       ros-humble-image-transport
Requires:       ros-humble-rclcpp
Requires:       ros-humble-robot-state-publisher
Requires:       ros-humble-ros-environment
Requires:       ros-humble-rviz-imu-plugin
Requires:       ros-humble-sensor-msgs
Requires:       ros-humble-std-msgs
Requires:       ros-humble-stereo-msgs
Requires:       ros-humble-vision-msgs
Requires:       ros-humble-xacro
Requires:       ros-humble-ros-workspace
BuildRequires:  opencv-devel
BuildRequires:  ros-humble-ament-cmake
BuildRequires:  ros-humble-camera-info-manager
BuildRequires:  ros-humble-cv-bridge
BuildRequires:  ros-humble-depthai
BuildRequires:  ros-humble-depthai-bridge
BuildRequires:  ros-humble-depthai-descriptions
BuildRequires:  ros-humble-depthai-ros-msgs
BuildRequires:  ros-humble-foxglove-msgs
BuildRequires:  ros-humble-image-transport
BuildRequires:  ros-humble-rclcpp
BuildRequires:  ros-humble-ros-environment
BuildRequires:  ros-humble-rviz-imu-plugin
BuildRequires:  ros-humble-sensor-msgs
BuildRequires:  ros-humble-std-msgs
BuildRequires:  ros-humble-stereo-msgs
BuildRequires:  ros-humble-vision-msgs
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
The depthai_examples package

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/humble" \
    -DAMENT_PREFIX_PATH="/opt/ros/humble" \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble" \
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
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Thu Aug 29 2024 Adam Serafin <adam.serafin@luxonis.com> - 2.10.0-1
- Autogenerated by Bloom

* Thu Jan 25 2024 sachin <sachin@luxonis.com> - 2.9.0-1
- Autogenerated by Bloom

* Tue Oct 17 2023 sachin <sachin@luxonis.com> - 2.8.2-1
- Autogenerated by Bloom

* Wed Sep 13 2023 sachin <sachin@luxonis.com> - 2.8.1-2
- Autogenerated by Bloom

* Wed Sep 13 2023 sachin <sachin@luxonis.com> - 2.8.1-1
- Autogenerated by Bloom

* Thu Sep 07 2023 sachin <sachin@luxonis.com> - 2.8.0-1
- Autogenerated by Bloom

* Thu Aug 10 2023 sachin <sachin@luxonis.com> - 2.7.5-1
- Autogenerated by Bloom

* Mon Jun 26 2023 sachin <sachin@luxonis.com> - 2.7.4-1
- Autogenerated by Bloom

* Fri Jun 16 2023 sachin <sachin@luxonis.com> - 2.7.3-1
- Autogenerated by Bloom

* Fri May 12 2023 sachin <sachin@luxonis.com> - 2.7.2-1
- Autogenerated by Bloom

* Thu Apr 06 2023 sachin <sachin@luxonis.com> - 2.7.1-1
- Autogenerated by Bloom

* Wed Mar 29 2023 sachin <sachin@luxonis.com> - 2.7.0-1
- Autogenerated by Bloom

* Mon Feb 27 2023 sachin <sachin@luxonis.com> - 2.6.4-1
- Autogenerated by Bloom

* Fri Feb 17 2023 sachin <sachin@luxonis.com> - 2.6.3-1
- Autogenerated by Bloom

* Tue Feb 07 2023 sachin <sachin@luxonis.com> - 2.6.2-1
- Autogenerated by Bloom

* Sun Jan 29 2023 sachin <sachin@luxonis.com> - 2.6.1-1
- Autogenerated by Bloom

* Thu Aug 11 2022 sachin <sachin@luxonis.com> - 2.5.3-1
- Autogenerated by Bloom

* Thu Jun 02 2022 sachin <sachin@luxonis.com> - 2.5.2-1
- Autogenerated by Bloom

