set(BOX2D_SOURCE_FILES
	b2/mmB2BlockAllocator.c
	b2/mmB2Body.c
	b2/mmB2BroadPhase.c
	b2/mmB2Collision.c
	b2/mmB2CollisionCircle.c
	b2/mmB2CollisionEdge.c
	b2/mmB2CollisionPolygons.c
	b2/mmB2Common.c
	b2/mmB2Contact.c
	b2/mmB2ContactChainCircle.c
	b2/mmB2ContactChainPolygon.c
	b2/mmB2ContactCircleCircle.c
	b2/mmB2ContactEdgeCircle.c
	b2/mmB2ContactEdgePolygon.c
	b2/mmB2ContactManager.c
	b2/mmB2ContactPolygonCircle.c
	b2/mmB2ContactPolygonPolygon.c
	b2/mmB2ContactSolver.c
	b2/mmB2Distance.c
	b2/mmB2Draw.c
	b2/mmB2DynamicTree.c
	b2/mmB2Fixture.c
	b2/mmB2GrowableStack.c
	b2/mmB2Island.c
	b2/mmB2Joint.c
	b2/mmB2JointDistance.c
	b2/mmB2JointFriction.c
	b2/mmB2JointGear.c
	b2/mmB2JointMotor.c
	b2/mmB2JointMouse.c
	b2/mmB2JointPrismatic.c
	b2/mmB2JointPulley.c
	b2/mmB2JointRevolute.c
	b2/mmB2JointTypes.c
	b2/mmB2JointWeld.c
	b2/mmB2JointWheel.c
	b2/mmB2Math.c
	b2/mmB2Rope.c
	b2/mmB2Settings.c
	b2/mmB2Shape.c
	b2/mmB2ShapeChain.c
	b2/mmB2ShapeCircle.c
	b2/mmB2ShapeEdge.c
	b2/mmB2ShapePolygon.c
	b2/mmB2ShapeTypes.c
	b2/mmB2StackAllocator.c
	b2/mmB2TimeOfImpact.c
	b2/mmB2Timer.c
	b2/mmB2World.c
	b2/mmB2WorldCallbacks.c)

set(BOX2D_HEADER_FILES
	b2/mmB2Api.h
	b2/mmB2BlockAllocator.h
	b2/mmB2Body.h
	b2/mmB2BroadPhase.h
	b2/mmB2Collision.h
	b2/mmB2Common.h
	b2/mmB2Contact.h
	b2/mmB2ContactChainCircle.h
	b2/mmB2ContactChainPolygon.h
	b2/mmB2ContactCircleCircle.h
	b2/mmB2ContactEdgeCircle.h
	b2/mmB2ContactEdgePolygon.h
	b2/mmB2ContactManager.h
	b2/mmB2ContactPolygonCircle.h
	b2/mmB2ContactPolygonPolygon.h
	b2/mmB2ContactSolver.h
	b2/mmB2Distance.h
	b2/mmB2Draw.h
	b2/mmB2DynamicTree.h
	b2/mmB2Fixture.h
	b2/mmB2GrowableStack.h
	b2/mmB2Header.h
	b2/mmB2Island.h
	b2/mmB2Joint.h
	b2/mmB2JointDistance.h
	b2/mmB2JointFriction.h
	b2/mmB2JointGear.h
	b2/mmB2JointMotor.h
	b2/mmB2JointMouse.h
	b2/mmB2JointPrismatic.h
	b2/mmB2JointPulley.h
	b2/mmB2JointRevolute.h
	b2/mmB2JointTypes.h
	b2/mmB2JointWeld.h
	b2/mmB2JointWheel.h
	b2/mmB2Math.h
	b2/mmB2MetaAllocator.h
	b2/mmB2Prefix.h
	b2/mmB2Rope.h
	b2/mmB2Settings.h
	b2/mmB2Shape.h
	b2/mmB2ShapeChain.h
	b2/mmB2ShapeCircle.h
	b2/mmB2ShapeEdge.h
	b2/mmB2ShapePolygon.h
	b2/mmB2ShapeTypes.h
	b2/mmB2StackAllocator.h
	b2/mmB2Suffix.h
	b2/mmB2TimeOfImpact.h
	b2/mmB2Timer.h
	b2/mmB2TimeStep.h
	b2/mmB2Types.h
	b2/mmB2World.h
	b2/mmB2WorldCallbacks.h)

add_library(box2d ${BOX2D_SOURCE_FILES} ${BOX2D_HEADER_FILES})
target_include_directories(box2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(box2d PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS) 

  set_target_properties(box2d PROPERTIES
    COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMPILE_PDB_NAME box2d
  )

  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/box2d.pdb"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    OPTIONAL
  )
endif()

if (BUILD_SHARED_LIBS)
  target_compile_definitions(box2d
    PUBLIC
      B2_SHARED
  )
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX2D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "include" FILES ${BOX2D_HEADER_FILES})

install(
  TARGETS box2d
  EXPORT box2dConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  EXPORT box2dConfig
  NAMESPACE box2d::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d"
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/box2dConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d"
)
