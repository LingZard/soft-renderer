add_rules("mode.debug", "mode.release")
set_languages("c++20")

add_requires("catch2 v3.8.1")
add_requires("minifb")

-- soft-renderer library target
target("soft-renderer")
    set_kind("static")
    add_files("src/**.cpp")
    add_includedirs("src")
    add_packages("minifb")
    set_languages("c++20")

-- pointcloud example
target("pointcloud")
    set_kind("binary")
    add_files("examples/pointcloud.cpp")
    add_deps("soft-renderer")
    add_includedirs(".")
    add_packages("minifb")
    set_languages("c++20")

-- wireframe example
target("wireframe")
    set_kind("binary")
    add_files("examples/wireframe.cpp")
    add_deps("soft-renderer")
    add_includedirs(".")
    add_packages("minifb")
    set_languages("c++20")

-- triangles example
target("triangles")
    set_kind("binary")
    add_files("examples/triangles.cpp")
    add_deps("soft-renderer")
    add_includedirs(".")
    add_packages("minifb")
    set_languages("c++20")

-- test target
target("tests")
    set_kind("binary")
    add_files("tests/*.cpp")
    add_deps("soft-renderer")
    add_packages("catch2")
    set_languages("c++20")
