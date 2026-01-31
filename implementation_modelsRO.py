def modelsRo(nprod, ndepot, ngarage, nstation, nvehicule, transtionCostMat, vehiculeFleet, depotMat, garageMat, stationMat):
    """
    Organiation des variables:
    nprod: le nombre de produit
    ndepot: Le nombre de dépôt
    ngarage: le nombre de garages
    nvehicule: Le nombre de véhicule
    transitionCostMat: la matrice des coûts de changeover, c'est une matice carrée
    vehiculeFleet: une liste de listes dont chaque line est la description d'un véhicule, la ligne i décrit le véhicule i
                    une line se présente comme suit:
                    [ID, capacite, ID_garage, initial_product]
    depotMat: une liste de listes contenant la description d'un dépôt
                    une ligne se présente comme suit:
                    [ID, (x, y), stock_P1, stock_P2, ..., stock_Pp]
    garageMat: une liste de listes contenant les description d'un garage
                    une ligne se présente comme suit:
                    [ID, (x, y)]
    stationMat: une liste de listes contenant la description d'une station service
                    une ligne se présente comme suit:
                    [ID, (x, y), demand_P1, demand_P2, ..., demand_Pp]
                    
    La fonction renvoie les matrices des variables

    """
    
    ## Définition de la matrice des distances pour ne pas calculer les distance à chaque fois
    #Un lieu sera représenté par son type, son id et ses coordonnées 
    # exemple [T, ID, (x,y)] T est le type: G pour garage, D pour dépôt et S pour station
    places_list = []
    
    for garage in garageMat:
        garage_id = garage[0]
        garage_x = garage[1]
        garage_y = garage[2]
        places_list.append([garage_id, (garage_x, garage_y)])
    
    for depot in depotMat:
        depot_id = depot[0]
        depot_x = depot[1]
        depot_y = depot[2]
        places_list.append([depot_id, (depot_x, depot_y)])
        
    for station in stationMat:
        station_id = station[0]
        station_x = station[1]
        station_y = station[2]
        places_list.append([station_id, (station_x, station_y)])
        
    
    return "Nonterminé"