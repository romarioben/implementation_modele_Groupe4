import math

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
        places_list.append(["G", garage_id, (garage_x, garage_y)])
    
    for depot in depotMat:
        depot_id = depot[0]
        depot_x = depot[1]
        depot_y = depot[2]
        places_list.append(["D", depot_id, (depot_x, depot_y)])
        
    for station in stationMat:
        station_id = station[0]
        station_x = station[1]
        station_y = station[2]
        places_list.append(["S", station_id, (station_x, station_y)])
    
    nbre_places_totales = len(places_list)
        
        
    ##Déterminer la matrice des distances
    distanceMat = []
    for place1 in places_list:
        pl_dis = []
        for place2 in places_list:
            pl_dis.append(math.sqrt((place1[2][0]-place2[2][0])**2+(place1[2][1]-place2[2][1])**2))
        distanceMat.append(pl_dis)
        
    ## Calculons d'abord le nombre d'étapes maximal T
    
    #Pour avoir le vehicule de capacité minimale
    min_vehi_cap = min(vehiculeFleet, key=lambda x : x[1])[1]
    
    #Pour avoir la station de demande maximale
    max_station_deman = max(max(stationMat, key=lambda x: max(x[1:]))[1:])
    
    #Calcule de T
    T = 2*(nprod * nstation * math.ceil(max_station_deman / min_vehi_cap)) + 1
        
    from ortools.sat.python import cp_model
    
    model = cp_model.CpModel()
    
    ##P_kt est la variable de postion des caminons
    P={}
    ##T_kt est la variable gardant le type de produit du camion k au moment t
    TC = {}
    ##QCA_kt represente la quantité de produit que comporte le camion au moment t
    QCA = {}
    
    
    for k in range(1, nvehicule+1):
        for t in range(0, T+1):
            P[(k,t)] = model.NewIntVar(lb=1, ub=nbre_places_totales, name=f"P_{k}_{t}")
            TC[(k,t)] = model.NewIntVar(1, nprod, name=f"TC_{k}_{t}")
            QCA[(k,t)] = model.NewIntVar(0, vehiculeFleet[k-1][1], name=f"QCA_{k}_{t}")
    
    #Initialisation des positions,
    for k in range(1, nvehicule+1):
        #Position initiale
        garage_id = vehiculeFleet[k-1][2]
        garage_index = next(i for i, place in enumerate(places_list) if place[0] == "G" and int(place[1]) == garage_id)
        model.Add(P[(k,0)] == garage_index + 1)  # +1 car les variables sont de 1 à nbre_places_totales
        
        #Type de produit initial
        initial_product = int(vehiculeFleet[k-1][3])
        model.Add(TC[(k,0)] == initial_product)
        
        #Quantité initiale
        model.Add(QCA[(k,0)] == 0)
        #Quantité finale
        model.Add(QCA[(k,T)] == 0)
    
    
    
    ##Définition des variables concernant les dépôts
    #QD_dpt est la quantité du produit du type p qu contient le dépôt d au moment t
    QD = {}
    
    
    for d in range(1, ndepot+1):
        for p in range(1, nprod+1):
            for t in range(T+1):
                QD[(d, p, t)] = model.NewIntVar(0, depotMat[d-1][p+2], f"QD_{d}_{p}_{t}")
    
    #Initialisation des stocks des dépôts
    for d in range(1, ndepot+1):
        for p in range(1, nprod+1):
            model.Add(QD[(d, p, 0)] == depotMat[d-1][p+2])
                
    ##Définition des variables concernant les stations
    #QS_spt est la quantité du produit du type p qu contient la station s au moment t
    QS = {}
    
    for s in range(1, nstation+1):
        for p in range(1, nprod+1):
            for t in range(T+1):
                QS[(s, p, t)] = model.NewIntVar(0, stationMat[s-1][p+2], f"QS_{s}_{p}_{t}")
    
    #Initialisation des demandes des stations et des stocks des dépôts
    for s in range(1, nstation+1):
        for p in range(1, nprod+1):
            model.Add(QS[(s, p, 0)] == 0)
            
    ##Ajout des contraintes et de la fonction objectif sera fait ici plus tard
    #La position d'un camion à t=0 doit être le même à t=T
    for k in range(1, nvehicule+1):
        model.Add(P[(k,0)] == P[(k,T)])
        
    #Contrainte sur la quantité finale du produit du type p dans la station s (lieu de livraison) 
    for s in range(1, nstation+1):
        for p in range(1, nprod+1):
            model.Add(QS[(s, p, T)] == stationMat[s-1][p+2])
    
    #Les contraintes logiques, si un camion k se retrouve sur un lieu de dépôt d
    #c’est que le volume qu’il transporte doit augmenter
    for k in range(1, nvehicule+1):
        for t in range(1, T):
            for d in range(1, ndepot+1):
                depot_id = depotMat[d-1][0]
                depot_index = next(i for i, place in enumerate(places_list) if place[0] == "D" and place[1] == depot_id)
                
                #print(type(P[(k,t)] == (depot_index + 1)))
                
                #une nouvelle variable booléenne pour indiquer si le camion k est au dépôt d au temps t
                is_at_depot = model.NewBoolVar(f"is_at_depot_{k}_{t}_{d}")

                # is_at_depot == 1  =>  P[k,t] == depot_index + 1
                model.Add(P[(k, t)] == (depot_index + 1)).OnlyEnforceIf(is_at_depot)
                # is_at_depot == 0  =>  P[k,t] != depot_index + 1
                model.Add(P[(k, t)] != (depot_index + 1)).OnlyEnforceIf(is_at_depot.Not())
                
                
                #Le volume dans le camion k au temps t+1 est égal au volume au temps t plus la quantité prise au dépôt d
                
                #variable booléenne pour indiquer si le camion k transporte le produit p au temps t
                is_transporting_product = model.NewBoolVar(f"is_transporting_product_{k}_{t}_{p}")
                
                # is_transporting_product == 1  =>  TC[k,t] == p
                model.Add(TC[(k, t)] == p).OnlyEnforceIf(is_transporting_product)
                # is_transporting_product == 0  =>  TC[k,t] != p
                model.Add(TC[(k, t)] != p).OnlyEnforceIf(is_transporting_product.Not())
                
                model.Add(QCA[(k,t)] -  QCA[(k,t-1)] ==  QD[(d,p,t-1)] - QD[(d,p,t-1)]).OnlyEnforceIf([is_at_depot, is_transporting_product])
                
                #une variable booléenne pour indiquer si le camion k change de type de produit entre t-1 et t
                is_changing_product = model.NewBoolVar(f"is_changing_product_{k}_{t}")
                model.Add(TC[(k,t)] != TC[(k,t-1)]).OnlyEnforceIf(is_changing_product)
                model.Add(TC[(k,t)] == TC[(k,t-1)]).OnlyEnforceIf(is_changing_product.Not())
                #Si un camion doit changer de type de produit  au niveau d’un dépôt il doit être vide
                model.Add(QCA[(k,t-1)] == 0).OnlyEnforceIf(is_at_depot, is_changing_product)
    
    ###Les contraintes logiques, si un camion k se retrouve sur un lieu de livraison station s:
    ##c’est que le volume qu’il transporte doit diminuer
    for k in range(1, nvehicule+1):
        for t in range(1, T):
            for s in range(1, nstation+1):
                station_id = stationMat[s-1][0]
                station_index = next(i for i, place in enumerate(places_list) if place[0] == "S" and place[1] == station_id)
                
                #une nouvelle variable booléenne pour indiquer si le camion k est au dépôt d au temps t
                is_at_station = model.NewBoolVar(f"is_at_station_{k}_{t}_{s}")

                # is_at_station == 1  =>  P[k,t] == station_index + 1
                model.Add(P[(k, t)] == (station_index + 1)).OnlyEnforceIf(is_at_station)
                # is_at_station == 0  =>  P[k,t] != station_index + 1
                model.Add(P[(k, t)] != (station_index + 1)).OnlyEnforceIf(is_at_station.Not())
                
                model.Add(QCA[(k,t)] < QCA[(k,t-1)]).OnlyEnforceIf(is_at_station)
                
                #variable booléenne pour indiquer si le camion k transporte le produit p au temps t
                is_transporting_product = model.NewBoolVar(f"is_transporting_product_{k}_{t}_{p}")
                
                # is_transporting_product == 1  =>  TC[k,t] == p
                model.Add(TC[(k, t)] == p).OnlyEnforceIf(is_transporting_product)
                # is_transporting_product == 0  =>  TC[k,t] != p
                model.Add(TC[(k, t)] != p).OnlyEnforceIf(is_transporting_product.Not())
                
                #Le volume dans le camion k au temps t+1 est égal au volume au temps t moins la quantité livrée à la station s
                model.Add(QCA[(k,t-1)] - QCA[(k,t)] == QS[(s,p,t)] - QS[(s,p,t-1)]).OnlyEnforceIf(is_at_station, is_transporting_product)
                
                #Un camion ne doit pas changer de type de produit au niveau d’une station
                model.Add(TC[(k,t)] == TC[(k,t-1)]).OnlyEnforceIf(is_at_station)

    ##Contrainte sur le volume d'un camion au garage au temps t>2
    for k in range(1, nvehicule+1):
        for t in range(3, T):
            garage_id = int(vehiculeFleet[k-1][2])
            garage_index = next(i for i, place in enumerate(places_list) if place[0] == "G" and int(place[1]) == garage_id)
            is_at_garage = model.NewBoolVar(f"is_at_garage_{k}_{t}")
            model.Add(P[(k,t)] == (garage_index + 1)).OnlyEnforceIf(is_at_garage)
            model.Add(P[(k,t)] != (garage_index + 1)).OnlyEnforceIf(is_at_garage.Not())
            model.Add(QCA[(k,t-1)] == 0).OnlyEnforceIf(is_at_garage)
            
            
    # --- Fonction objectif: minimise coût de changeover et distance ---
    # def add_objective_min_changeover_distance(model, P, TC, transtionCostMat, distanceMat, nprod, nbre_places_totales, nvehicule, T, scale=1000):
    #     """Ajoute à `model` une fonction objectif qui minimise:
    #     - les coûts de changeover (transitionCostMat)
    #     - la distance parcourue (distanceMat)
    #     Les coûts et distances sont multipliés par `scale` et arrondis en entiers.
    #     """
    #     # Mise à l'échelle et conversion en entiers (CP-SAT travaille mieux avec entiers)
    #     cost_scaled = [[int(round(c * scale)) for c in row] for row in transtionCostMat]
    #     dist_scaled = [[int(round(d * scale)) for d in row] for row in distanceMat]

    #     total_terms = []

    #     for k in range(1, nvehicule+1):
    #         for t in range(1, T+1):
    #             # Variables booléennes pour les transitions de produit (p_prev -> p_curr)
    #             b_trans = {}
    #             for p in range(1, nprod+1):
    #                 for q in range(1, nprod+1):
    #                     b = model.NewBoolVar(f"b_trans_k{k}_t{t}_p{p}_q{q}")
    #                     b_trans[(p,q)] = b
    #                     # Réification: b => (TC_prev == p AND TC_curr == q)
    #                     model.Add(TC[(k,t-1)] == p).OnlyEnforceIf(b)
    #                     model.Add(TC[(k,t-1)] != p).OnlyEnforceIf(b.Not())
    #                     model.Add(TC[(k,t)] == q).OnlyEnforceIf(b)
    #                     model.Add(TC[(k,t)] != q).OnlyEnforceIf(b.Not())
    #             # exactement une paire (p,q) vraie
    #             model.Add(sum(b_trans.values()) == 1)
    #             # coût de changement (peut être 0 si p==q)
    #             for (p,q), b in b_trans.items():
    #                 total_terms.append(cost_scaled[p-1][q-1] * b)

    #             # Variables booléennes pour le déplacement (i -> j)
    #             b_pos = {}
    #             for i in range(1, nbre_places_totales+1):
    #                 for j in range(1, nbre_places_totales+1):
    #                     b = model.NewBoolVar(f"b_pos_k{k}_t{t}_i{i}_j{j}")
    #                     b_pos[(i,j)] = b
    #                     model.Add(P[(k,t-1)] == i).OnlyEnforceIf(b)
    #                     model.Add(P[(k,t-1)] != i).OnlyEnforceIf(b.Not())
    #                     model.Add(P[(k,t)] == j).OnlyEnforceIf(b)
    #                     model.Add(P[(k,t)] != j).OnlyEnforceIf(b.Not())
    #             model.Add(sum(b_pos.values()) == 1)
    #             for (i,j), b in b_pos.items():
    #                 total_terms.append(dist_scaled[i-1][j-1] * b)

    #     # Minimiser la somme des coûts mis à l'échelle
    #     model.Minimize(sum(total_terms))

    # # Appel de la fonction objectif (scale=100 -> deux décimales conservées)
    # add_objective_min_changeover_distance(model, P, TC, transtionCostMat, distanceMat, nprod, nbre_places_totales, nvehicule, T, scale=1000)

    solver = cp_model.CpSolver()
    status = solver.Solve(model)
    print(f"Status: {solver.StatusName(status)}")
    
    #Afficher les positions des camions
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        for k in range(1, nvehicule+1):
            print(f"Trajet du camion {k}:")
            for t in range(0, T+1):
                pos_index = solver.Value(P[(k,t)]) - 1
                pos = places_list[pos_index]
                prod_type = solver.Value(TC[(k,t)])
                quantity = solver.Value(QCA[(k,t)])
                print(f"  Temps {t}: Position {pos[0]}-{pos[1]} at ({pos[2][0]}, {pos[2][1]}), Produit Type: {prod_type}, Quantité: {quantity}")
            print()
    
    
    #Aficher la quantité des stations par produit et par temps
    for s in range(1, nstation+1):  
        print(f"Stock de la station {s}:")
        for p in range(1, nprod+1):
            print(f"  Produit {p}: ", end="")
            for t in range(0, T+1):
                quantity = solver.Value(QS[(s, p, t)])
                print(f"T{t}:{quantity} ", end="")
            print()
        print()
    
    #Afficher la quantité des dépôts par produit et par temps
    for d in range(1, ndepot+1):  
        print(f"Stock du dépôt {d}:")
        for p in range(1, nprod+1):
            print(f"  Produit {p}: ", end="")
            for t in range(0, T+1):
                quantity = solver.Value(QD[(d, p, t)])
                print(f"T{t}:{quantity} ", end="")
            print()
        print()
    return "parfait"


nprod = 2
ndepot= 1
ngarage = 2
transtionCostMat = [[0, 18.1], [61.5, 0]]
vehiculeFleet = [["1", 20000, 1, 1], ["2", 20000, 1, 2]]
nstation = 3
nvehicule = 2
depotMat = [["1", 81.6, 63.6, 57914, 82626]]
garageMat = [["1", 98.1, 49.6], ["2", 56.8, 26]]
stationMat = [["1", 23.5, 42.2, 0, 4278], ["2", 3.5, 38.3, 1344, 2350], ["3", 56.7, 31.0, 0, 2319]]

result = modelsRo(nprod, ndepot, ngarage, nstation, nvehicule, transtionCostMat, vehiculeFleet, depotMat, garageMat, stationMat)
print(result)
