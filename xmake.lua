add_rules("mode.debug", "mode.release")
set_languages("c++20")

add_requires("catch2 v3.8.1")
add_requires("minifb")

-- main app target
target("app")
    set_kind("binary")
    add_files("src/*.cpp")
    add_packages("minifb")
    set_languages("c++20")

-- test target
target("tests")
    set_kind("binary")
    add_files("tests/*.cpp")
    add_deps("app")
    add_packages("catch2")
    set_languages("c++20")
