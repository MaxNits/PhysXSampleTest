#include <time.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <ctype.h>
#include <stdlib.h>
#include <cstring>
#include <PxPhysicsAPI.h>

#include "characterkinematic/PxExtended.h"
#include "characterkinematic/PxController.h"
#include "characterkinematic/PxBoxController.h"
#include "characterkinematic/PxCapsuleController.h"
#include "characterkinematic/PxControllerManager.h"

#include "vehicle/PxVehicleUtil.h"
#include "../SnippetVehicleCommon/SnippetVehicleRaycast.h"
#include "../SnippetVehicleCommon/SnippetVehicleFilterShader.h"
#include "../SnippetVehicleCommon/SnippetVehicleTireFriction.h"
#include "../SnippetVehicleCommon/SnippetVehicleCreate.h"

#include "../SnippetCommon/SnippetPrint.h"
#include "../SnippetCommon/SnippetPVD.h"
#include "../SnippetUtils/SnippetUtils.h"

#ifdef _DEBUG //If in 'Debug' load libraries for debug mode
#ifdef _WIN64
#pragma comment(lib, "PhysX3DEBUG_x64.lib")        //Always be needed  
#pragma comment(lib, "PhysX3CommonDEBUG_x64.lib")    //Always be needed
#pragma comment(lib, "PhysX3CookingDEBUG_x64.lib")
#else //Else load libraries for 'Release' mode
#pragma comment(lib, "PhysX3DEBUG_x86.lib")        //Always be needed  
#pragma comment(lib, "PhysX3CommonDEBUG_x86.lib")    //Always be needed
#pragma comment(lib, "PhysX3CookingDEBUG_x86.lib")
#endif
#pragma comment(lib, "PhysX3ExtensionsDEBUG.lib")    //PhysX extended library 
#pragma comment(lib, "PhysXVisualDebuggerSDKDEBUG.lib") //For PVD only
#else //Else load libraries for 'Release' mode
#ifdef _WIN64
#pragma comment(lib, "PhysX3_x64.lib")  
#pragma comment(lib, "PhysX3Common_x64.lib")
#pragma comment(lib, "PhysX3Cooking_x64.lib")
#else //Else load libraries for 'Release' mode
#pragma comment(lib, "PhysX3_x86.lib")  
#pragma comment(lib, "PhysX3Common_x86.lib")
#pragma comment(lib, "PhysX3Cooking_x86.lib")
#endif
#pragma comment(lib, "PhysX3Extensions.lib")    //PhysX extended library 
#pragma comment(lib, "PhysXVisualDebuggerSDK.lib") //For PVD only 
#endif

using namespace std;
using namespace physx;

PxReal gTimeStep = 1.0f/60.0f;
const int MOUNTAIN_COUNT = 20;
const int HEIGHT_SCALE = 100;
int COLUMNS = 1000;
int ROWS = 1000;
PxHeightFieldSample* sampleHeights = NULL;

PxControllerManager* controllerManager = NULL;
PxCapsuleController* capsuleController = NULL;
PxControllerFilters characterControllerFilters;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxCooking*				gCooking	= NULL;

PxMaterial*				gMaterial	= NULL;

PxVisualDebuggerConnection*		 
						gConnection	= NULL;

VehicleSceneQueryData*	gVehicleSceneQueryData = NULL;
PxBatchQuery*			gBatchQuery = NULL;

PxVehicleDrivableSurfaceToTireFrictionPairs* gFrictionPairs = NULL;

PxRigidStatic*			gGroundPlane = NULL;
PxVehicleDrive4W*		gVehicle4W		= NULL;

bool					gIsVehicleInAir = true;


void initPhysX();
void HeightFieldInit();
void VehicleCreation(PxPhysics& physics);
void stepPhysX();
void ConnectPVD(); // generating Visual Debugger file
void ShutdowPhysX();
void createVehicle4WSimulationData(const PxF32 chassisMass, PxConvexMesh* chassisConvexMesh, const PxF32 wheelMass, 
								   PxConvexMesh** wheelConvexMeshes, const PxVec3* wheelCentreOffsets,
								   PxVehicleWheelsSimData& wheelsData, PxVehicleDriveSimData4W& driveData, PxVehicleChassisData& chassisData);

PxF32 gSteerVsForwardSpeedData[2*8]=
{
	0.0f,		0.75f,
	5.0f,		0.75f,
	30.0f,		0.125f,
	120.0f,		0.1f,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32,
	PX_MAX_F32, PX_MAX_F32
};
PxFixedSizeLookupTable<8> gSteerVsForwardSpeedTable(gSteerVsForwardSpeedData,4);

PxVehicleKeySmoothingData gKeySmoothingData=
{
	{
		6.0f,	//rise rate eANALOG_INPUT_ACCEL
		6.0f,	//rise rate eANALOG_INPUT_BRAKE		
		6.0f,	//rise rate eANALOG_INPUT_HANDBRAKE	
		2.5f,	//rise rate eANALOG_INPUT_STEER_LEFT
		2.5f,	//rise rate eANALOG_INPUT_STEER_RIGHT
	},
	{
		10.0f,	//fall rate eANALOG_INPUT_ACCEL
		10.0f,	//fall rate eANALOG_INPUT_BRAKE		
		10.0f,	//fall rate eANALOG_INPUT_HANDBRAKE	
		5.0f,	//fall rate eANALOG_INPUT_STEER_LEFT
		5.0f	//fall rate eANALOG_INPUT_STEER_RIGHT
	}
};

PxVehiclePadSmoothingData gPadSmoothingData=
{
	{
		6.0f,	//rise rate eANALOG_INPUT_ACCEL
		6.0f,	//rise rate eANALOG_INPUT_BRAKE		
		6.0f,	//rise rate eANALOG_INPUT_HANDBRAKE	
		2.5f,	//rise rate eANALOG_INPUT_STEER_LEFT
		2.5f,	//rise rate eANALOG_INPUT_STEER_RIGHT
	},
	{
		10.0f,	//fall rate eANALOG_INPUT_ACCEL
		10.0f,	//fall rate eANALOG_INPUT_BRAKE		
		10.0f,	//fall rate eANALOG_INPUT_HANDBRAKE	
		5.0f,	//fall rate eANALOG_INPUT_STEER_LEFT
		5.0f	//fall rate eANALOG_INPUT_STEER_RIGHT
	}
};

PxVehicleDrive4WRawInputData gVehicleInputData;

enum DriveMode
{
	eDRIVE_MODE_ACCEL_FORWARDS=0,
	eDRIVE_MODE_ACCEL_REVERSE,
	eDRIVE_MODE_HARD_TURN_LEFT,
	eDRIVE_MODE_HANDBRAKE_TURN_LEFT,
	eDRIVE_MODE_HARD_TURN_RIGHT,
	eDRIVE_MODE_HANDBRAKE_TURN_RIGHT,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_NONE
};

DriveMode gDriveModeOrder[] =
{
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_ACCEL_REVERSE,
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_HARD_TURN_LEFT, 
	eDRIVE_MODE_BRAKE,
	eDRIVE_MODE_HARD_TURN_RIGHT,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_HANDBRAKE_TURN_LEFT,
	eDRIVE_MODE_ACCEL_FORWARDS,
	eDRIVE_MODE_HANDBRAKE_TURN_RIGHT,
	eDRIVE_MODE_NONE
};

PxF32					gVehicleModeLifetime = 4.0f;
PxF32					gVehicleModeTimer = 0.0f;
PxU32					gVehicleOrderProgress = 0;
bool					gVehicleOrderComplete = false;
bool					gMimicKeyInputs = false;

VehicleDesc initVehicleDesc()
{
	//Set up the chassis mass, dimensions, moment of inertia, and center of mass offset.
	//The moment of inertia is just the moment of inertia of a cuboid but modified for easier steering.
	//Center of mass offset is 0.65m above the base of the chassis and 0.25m towards the front.
	const PxF32 chassisMass = 1500.0f;
	const PxVec3 chassisDims(2.5f,2.0f,5.0f);
	const PxVec3 chassisMOI
		((chassisDims.y*chassisDims.y + chassisDims.z*chassisDims.z)*chassisMass/12.0f,
		 (chassisDims.x*chassisDims.x + chassisDims.z*chassisDims.z)*0.8f*chassisMass/12.0f,
		 (chassisDims.x*chassisDims.x + chassisDims.y*chassisDims.y)*chassisMass/12.0f);
	const PxVec3 chassisCMOffset(0.0f, -chassisDims.y*0.5f + 0.65f, 0.25f);

	//Set up the wheel mass, radius, width, moment of inertia, and number of wheels.
	//Moment of inertia is just the moment of inertia of a cylinder.
	const PxF32 wheelMass = 20.0f;
	const PxF32 wheelRadius = 0.5f;
	const PxF32 wheelWidth = 0.4f;
	const PxF32 wheelMOI = 0.5f*wheelMass*wheelRadius*wheelRadius;
	const PxU32 nbWheels = 6;

	VehicleDesc vehicleDesc;
	vehicleDesc.chassisMass = chassisMass;
	vehicleDesc.chassisDims = chassisDims;
	vehicleDesc.chassisMOI = chassisMOI;
	vehicleDesc.chassisCMOffset = chassisCMOffset;
	vehicleDesc.chassisMaterial = gMaterial;
	vehicleDesc.wheelMass = wheelMass;
	vehicleDesc.wheelRadius = wheelRadius;
	vehicleDesc.wheelWidth = wheelWidth;
	vehicleDesc.wheelMOI = wheelMOI;
	vehicleDesc.numWheels = nbWheels;
	vehicleDesc.wheelMaterial = gMaterial;
	return vehicleDesc;
}

void startAccelerateForwardsMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(true);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(1.0f);
	}
}

void startAccelerateReverseMode()
{
	gVehicle4W->mDriveDynData.forceGearChange(PxVehicleGearsData::eREVERSE);

	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(true);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(1.0f);
	}
}

void startBrakeMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalBrake(true);
	}
	else
	{
		gVehicleInputData.setAnalogBrake(1.0f);
	}
}

void startTurnHardLeftMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(true);
		gVehicleInputData.setDigitalSteerLeft(true);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(true);
		gVehicleInputData.setAnalogSteer(-1.0f);
	}
}

void startTurnHardRightMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(true);
		gVehicleInputData.setDigitalSteerRight(true);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(1.0f);
		gVehicleInputData.setAnalogSteer(1.0f);
	}
}

void startHandbrakeTurnLeftMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalSteerLeft(true);
		gVehicleInputData.setDigitalHandbrake(true);
	}
	else
	{
		gVehicleInputData.setAnalogSteer(-1.0f);
		gVehicleInputData.setAnalogHandbrake(1.0f);
	}
}

void startHandbrakeTurnRightMode()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalSteerRight(true);
		gVehicleInputData.setDigitalHandbrake(true);
	}
	else
	{
		gVehicleInputData.setAnalogSteer(1.0f);
		gVehicleInputData.setAnalogHandbrake(1.0f);
	}
}

void releaseAllControls()
{
	if(gMimicKeyInputs)
	{
		gVehicleInputData.setDigitalAccel(false);
		gVehicleInputData.setDigitalSteerLeft(false);
		gVehicleInputData.setDigitalSteerRight(false);
		gVehicleInputData.setDigitalBrake(false);
		gVehicleInputData.setDigitalHandbrake(false);
	}
	else
	{
		gVehicleInputData.setAnalogAccel(0.0f);
		gVehicleInputData.setAnalogSteer(0.0f);
		gVehicleInputData.setAnalogBrake(0.0f);
		gVehicleInputData.setAnalogHandbrake(0.0f);
	}
}

void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxProfileZoneManager* profileZoneManager = &PxProfileZoneManager::createProfileZoneManager(gFoundation);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true, profileZoneManager);

	if(gPhysics->getPvdConnectionManager())
	{
		gPhysics->getVisualDebugger()->setVisualizeConstraints(true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS, true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_SCENEQUERIES, true);
		gConnection = PxVisualDebuggerExt::createConnection(gPhysics->getPvdConnectionManager(), PVD_HOST, 5425, 10);
	}

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f * 5, 0.0f);
	
	PxU32 numWorkers = 1;
	gDispatcher = PxDefaultCpuDispatcherCreate(numWorkers);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= VehicleFilterShader;

	gScene = gPhysics->createScene(sceneDesc);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	gCooking = 	PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));

	PxInitVehicleSDK(*gPhysics);
	PxVehicleSetBasisVectors(PxVec3(0,1,0), PxVec3(0,0,1));
	PxVehicleSetUpdateMode(PxVehicleUpdateMode::eVELOCITY_CHANGE);

	//Create the batched scene queries for the suspension raycasts.
	gVehicleSceneQueryData = VehicleSceneQueryData::allocate(1, PX_MAX_NB_WHEELS, 1, gAllocator);
	gBatchQuery = VehicleSceneQueryData::setUpBatchedSceneQuery(0, *gVehicleSceneQueryData, gScene);

	//Create the friction table for each combination of tire and surface type.
	gFrictionPairs = createFrictionPairs(gMaterial);

	//Create a vehicle that will drive on the plane.
	VehicleDesc vehicleDesc = initVehicleDesc();
	gVehicle4W = createVehicle4W(vehicleDesc, gPhysics, gCooking);
	PxTransform startTransform(PxVec3(-10, (vehicleDesc.chassisDims.y*0.5f + vehicleDesc.wheelRadius + 1.0f) + 230, -10), PxQuat(PxIdentity));
	gVehicle4W->getRigidDynamicActor()->setGlobalPose(startTransform);
	gScene->addActor(*gVehicle4W->getRigidDynamicActor());

	HeightFieldInit();

	//Set the vehicle to rest in first gear.
	//Set the vehicle to use auto-gears.
	gVehicle4W->setToRestState();
	gVehicle4W->mDriveDynData.forceGearChange(PxVehicleGearsData::eFIRST);
	gVehicle4W->mDriveDynData.setUseAutoGears(true);

	gVehicleModeTimer = 0.0f;
	gVehicleOrderProgress = 0;
	startBrakeMode();
}

void HeightFieldInit()
{
	std::ifstream infile("D:/Java_array.txt");
	//short* height;
	int number, i = 0;
	//int rows = 0;
	//int columns = 0;

	infile >> ROWS >> COLUMNS;
	sampleHeights = new PxHeightFieldSample[ROWS * COLUMNS];

	while (infile >> number) {
		sampleHeights[i].height = number;
		//sampleHeights[i].height = 100;
		sampleHeights[i].materialIndex0 = 1;
		sampleHeights[i].materialIndex1 = 2;
		i++;
	}

	PxHeightFieldDesc* heightFieldDesc = new PxHeightFieldDesc();
	heightFieldDesc->nbColumns = COLUMNS;
	heightFieldDesc->nbRows = ROWS;
	heightFieldDesc->thickness = -10;
	heightFieldDesc->convexEdgeThreshold = 3;
	heightFieldDesc->samples.data = sampleHeights;
	heightFieldDesc->flags = PxHeightFieldFlag::eNO_BOUNDARY_EDGES;
	heightFieldDesc->samples.stride = sizeof(PxHeightFieldSample);

	PxHeightField* heightField = gPhysics->createHeightField(*heightFieldDesc);
	PxReal heightScale = 1.0f;
	PxReal rowScale = 1.0f;
	PxReal columnScale = 1.0f;
	PxHeightFieldGeometry* hfGeom = new PxHeightFieldGeometry(heightField, PxMeshGeometryFlags(), heightScale, rowScale, columnScale);
	PxMaterial* material = gPhysics->createMaterial(0.5, 0.5, 0.6);
	PxTransform planePos = PxTransform(PxVec3((float)(- ROWS / 2), 0.0f, (float)(- COLUMNS / 2)), PxQuat(0, PxVec3(0.0f, 0.0f, 0.0f)));
	
	gGroundPlane = gPhysics->createRigidStatic(planePos);
	PxShape* heightFieldShape = gGroundPlane->createShape(*hfGeom, *material);
	
		cout << "Setting filter for shapes 1" << endl;
	//PxShape* shapes[10];
	//gGroundPlane->getShapes(shapes, 10);
		cout << "Setting filter for shapes 2" << endl;
	//Set the query filter data of the ground plane so that the vehicle raycasts can hit the ground.
	PxFilterData qryFilterData;
		cout << "Setting filter for shapes 2.1" << endl;
	setupDrivableSurface(qryFilterData);
		cout << "Setting filter for shapes 2.2" << endl;
	heightFieldShape->setQueryFilterData(qryFilterData);
		cout << "Setting filter for shapes 3" << endl;
	//Set the simulation filter data of the ground plane so that it collides with the chassis of a vehicle but not the wheels.
	PxFilterData simFilterData;
	simFilterData.word0 = COLLISION_FLAG_GROUND;
	simFilterData.word1 = COLLISION_FLAG_GROUND_AGAINST;
	heightFieldShape->setSimulationFilterData(simFilterData);
		cout << "Setting filter for shapes 4" << endl;
	
	gScene->addActor(*gGroundPlane);
		cout << "Setting filter for shapes 5" << endl;
}


void incrementDrivingMode(const PxF32 timestep)
{
	gVehicleModeTimer += timestep;
	if(gVehicleModeTimer > gVehicleModeLifetime)
	{
		//If the mode just completed was eDRIVE_MODE_ACCEL_REVERSE then switch back to forward gears.
		if(eDRIVE_MODE_ACCEL_REVERSE == gDriveModeOrder[gVehicleOrderProgress])
		{
			gVehicle4W->mDriveDynData.forceGearChange(PxVehicleGearsData::eFIRST);
		}

		//Increment to next driving mode.
		gVehicleModeTimer = 0.0f;
		gVehicleOrderProgress++;
		releaseAllControls();

		//If we are at the end of the list of driving modes then start again.
		if(eDRIVE_MODE_NONE == gDriveModeOrder[gVehicleOrderProgress])
		{
			gVehicleOrderProgress = 0;
			gVehicleOrderComplete = true;
		}

		//Start driving in the selected mode.
		DriveMode eDriveMode = gDriveModeOrder[gVehicleOrderProgress];
		switch(eDriveMode)
		{
		case eDRIVE_MODE_ACCEL_FORWARDS:
			startAccelerateForwardsMode();
			break;
		case eDRIVE_MODE_ACCEL_REVERSE:
			startAccelerateReverseMode();
			break;
		case eDRIVE_MODE_HARD_TURN_LEFT:
			startTurnHardLeftMode();
			break;
		case eDRIVE_MODE_HANDBRAKE_TURN_LEFT:
			startHandbrakeTurnLeftMode();
			break;
		case eDRIVE_MODE_HARD_TURN_RIGHT:
			startTurnHardRightMode();
			break;
		case eDRIVE_MODE_HANDBRAKE_TURN_RIGHT:
			startHandbrakeTurnRightMode();
			break;
		case eDRIVE_MODE_BRAKE:
			startBrakeMode();
			break;
		case eDRIVE_MODE_NONE:
			break;
		};

		//If the mode about to start is eDRIVE_MODE_ACCEL_REVERSE then switch to reverse gears.
		if(eDRIVE_MODE_ACCEL_REVERSE == gDriveModeOrder[gVehicleOrderProgress])
		{
			gVehicle4W->mDriveDynData.forceGearChange(PxVehicleGearsData::eREVERSE);
		}
	}
}

void stepPhysics()
{
	cout << "stepPhysics() 1" << endl;
	const PxF32 timestep = 1.0f/60.0f;

	//Cycle through the driving modes to demonstrate how to accelerate/reverse/brake/turn etc.
	incrementDrivingMode(timestep);
	//Update the control inputs for the vehicle.
	if(gMimicKeyInputs)
	{
		cout << "stepPhysics() 2" << endl;
		PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs(gKeySmoothingData, gSteerVsForwardSpeedTable, gVehicleInputData, timestep, gIsVehicleInAir, *gVehicle4W);
	}
	else
	{
		cout << "stepPhysics() 3" << endl;
		PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs(gPadSmoothingData, gSteerVsForwardSpeedTable, gVehicleInputData, timestep, gIsVehicleInAir, *gVehicle4W);
	}
	//Raycasts.
	cout << "stepPhysics() 4" << endl;
	PxVehicleWheels* vehicles[1] = {gVehicle4W};
	PxRaycastQueryResult* raycastResults = gVehicleSceneQueryData->getRaycastQueryResultBuffer(0);
	const PxU32 raycastResultsSize = gVehicleSceneQueryData->getRaycastQueryResultBufferSize();
	PxVehicleSuspensionRaycasts(gBatchQuery, 1, vehicles, raycastResultsSize, raycastResults);
	//Vehicle update.
	cout << "stepPhysics() 5" << endl;
	const PxVec3 grav = gScene->getGravity();
	cout << "stepPhysics() 5.1" << endl;
	PxWheelQueryResult wheelQueryResults[PX_MAX_NB_WHEELS];
	cout << "stepPhysics() 5.2" << endl;
	PxVehicleWheelQueryResult vehicleQueryResults[1] = {{wheelQueryResults, gVehicle4W->mWheelsSimData.getNbWheels()}};
	cout << "stepPhysics() 5.3" << endl;
	PxVehicleUpdates(timestep, grav, *gFrictionPairs, 1, vehicles, vehicleQueryResults);
	//Work out if the vehicle is in the air.
	cout << "stepPhysics() 6" << endl;
	gIsVehicleInAir = gVehicle4W->getRigidDynamicActor()->isSleeping() ? false : PxVehicleIsInAir(vehicleQueryResults[0]);
	//capsuleController->move(PxVec3(10.0f, -9.85f * 2.0f, 10.0f), 0.1f, gTimeStep, PxControllerFilters());
	//Scene update.
	cout << "stepPhysics() 7" << endl;
	gScene->simulate(timestep);
	gScene->fetchResults(true);
	cout << "stepPhysics() 8" << endl;
}
	
void cleanupPhysics()
{
	gVehicle4W->getRigidDynamicActor()->release();
	gVehicle4W->free();
	gGroundPlane->release();
	gBatchQuery->release();
	gVehicleSceneQueryData->free(gAllocator);
	gFrictionPairs->release();
	PxCloseVehicleSDK();

	gMaterial->release();
	gCooking->release();
	gScene->release();
	gDispatcher->release();
	PxProfileZoneManager* profileZoneManager = gPhysics->getProfileZoneManager();
	if(gConnection != NULL)
		gConnection->release();
	gPhysics->release();	
	profileZoneManager->release();
	gFoundation->release();

	cout << "SnippetVehicle4W done.\n" << endl;
}

void keyPress(const char key, const PxTransform& camera)
{
	PX_UNUSED(camera);
	PX_UNUSED(key);
}

int main()
{
	srand ((unsigned int)time(NULL));
	cout << "Simulation started." << endl;
	initPhysics();
	cout << "Simulation 1" << endl;
	ConnectPVD();
	cout << "Simulation 2" << endl;

	while(!gVehicleOrderComplete)
	{
		stepPhysics();
	}
	cleanupPhysics();	
	cout << "Simulation ended." << endl;
}

void ConnectPVD()
{	
	if(gPhysics->getPvdConnectionManager() == NULL)
		return;
	const char* filename = "D:\\PvdCaptureTest.pxd2";
	PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();
	debugger::comm::PvdConnection* theConnection = PxVisualDebuggerExt::createConnection(gPhysics->getPvdConnectionManager(), filename, connectionFlags);
}

void ShutdownPhysX()
{
	//PxCloseVehicleSDK();
	gPhysics->release();
	gFoundation->release();
	controllerManager->release();
	delete [] sampleHeights;
}