===========================================================
README - Benchmark Instances for MCmpt-VRPTW with Reconfigurable Vehicles
===========================================================

This file describes the structure and format of the instance files used in our experimental study. Each instance includes detailed information on vehicle fleet characteristics, frigobox constraints, and customer demands. The instance format is plain-text and organized into three main sections.

-----------------------------------------------------------
1. VEHICLE FLEET DESCRIPTION
-----------------------------------------------------------
This section lists all available vehicles, their types, compartment capacities, and the maximum number of frigoboxes each can carry.

FORMAT:
VEHICLE_ID    TYPE       CAPACITY_FRESH  CAPACITY_FROZEN  MAX_FRIGOBOXES

- VEHICLE_ID:    Identifier (can be numeric or alphanumeric)
- TYPE:          {Fresh, Frozen, Mixed}
- CAPACITY_FRESH: Capacity dedicated to fresh products (set to 0 for frozen-only vehicles)
- CAPACITY_FROZEN: Capacity dedicated to frozen products (set to 0 for fresh-only vehicles)
- MAX_FRIGOBOXES: Max number of modular frigoboxes that can be loaded (0 if not allowed)

EXAMPLE:
1             Fresh               150              0                1  
2             Frozen                0            200                1  
O2            Mixed                75             75                0  

-----------------------------------------------------------
2. FRIGOBOX CONFIGURATION
-----------------------------------------------------------
This section defines the global constraints on frigobox usage across the fleet.

FIELDS:
- TOTAL_AVAILABLE_FRIGOBOXES: The total number of frigoboxes available at the depot
- CAPACITY_PER_FRIGOBOX: Frigobox capacity (in volume) 
- WEIGHT_PER_FRIGOBOX: Physical volume of each frigobox 

EXAMPLE:
TOTAL_AVAILABLE_FRIGOBOXES: 15  
CAPACITY_PER_FRIGOBOX: 40  
WEIGHT_PER_FRIGOBOX: 40  

-----------------------------------------------------------
3. CUSTOMER INFORMATION
-----------------------------------------------------------
This section defines the customer data including spatial coordinates, time window constraints, and product demand.

FORMAT:
CUST_NO	XCOORD	YCOORD	READY_TIME	DUE_DATE	SERVICE_TIME	FROZEN	FRESH

- CUST_NO:       Customer index (0 typically represents the depot)
- XCOORD/YCOORD: Cartesian coordinates
- READY_TIME:    Earliest start time for service
- DUE_DATE:      Latest allowable time for service
- SERVICE_TIME:  Duration of service at the customer
- FROZEN:        Frozen product demand
- FRESH:         Fresh product demand

EXAMPLE:
0	70	70	0	1351	0	0	0       (Depot)
1	33	78	750	809	90	6	14     (Customer 1)

-----------------------------------------------------------
NOTES:
-----------------------------------------------------------
- All time units are assumed to be in minutes.
- Coordinates are in Euclidean space (for distance computation).
- Each instance assumes the use of heterogeneous and/or mixed-reconfigurable vehicles.

-----------------------------------------------------------
CONTACT:
-----------------------------------------------------------
For more information, questions, or instance generation scripts, please contact the authors or refer to the accompanying journal article.

