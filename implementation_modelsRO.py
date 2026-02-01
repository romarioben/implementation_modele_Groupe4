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
        
        
    # conversion en entiers (préserve décimales via scale
    scale = 1000
    cost_scaled = [[int(round(c * scale)) for c in row] for row in transtionCostMat]
    dist_scaled = [[int(round(d * scale)) for d in row] for row in distanceMat]
        
    ## Calculons d'abord le nombre d'étapes maximal T
    
    #Pour avoir le vehicule de capacité minimale
    max_vehi_cap = max(vehiculeFleet, key=lambda x : x[1])[1]
    
    #Pour avoir la station de demande maximale
    max_station_deman = max(max(stationMat, key=lambda x: max(x[1:]))[1:])
    
    #Calcule de T
    T = 2*(nprod * nstation * math.ceil(max_station_deman / max_vehi_cap)) + 1
    
    
    T = (T//nvehicule) + 1
    # maxT = (T//nvehicule) + 1  # Ajustement de T en fonction du nombre de véhicules
    # ##Modelisation du problème
    
    # minT = math.ceil(max_station_deman / max_vehi_cap) * nstation  + 2 
    
    # if minT > maxT:
    #     minT , maxT = maxT, minT  # échanger si nécessaire
    
    # print("minT:", minT, "maxT:", maxT)
    print("T fixé à:", T)
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
                

    
    
    #Initialisation des quantités des stations et des stocks des dépôts
    for s in range(1, nstation+1):
        for p in range(1, nprod+1):
            model.Add(QS[(s, p, 0)] == 0)
            #Contrainte des quantié finale des stations
            model.Add(QS[(s, p, T)] == stationMat[s-1][p+2])
    
    ##Contraintes modelisant qu'un camion qui quitte un garage doit aller à un dépôt
    garage_indices = [i for i, place in enumerate(places_list) if place[0] == "G"]
    depot_indices = [i for i, place in enumerate(places_list) if place[0] == "D"]

    for k in range(1, nvehicule+1):
        for t in range(1, T+1):
            for g in garage_indices:
                is_at_g = model.NewBoolVar(f"is_at_g_k{k}_t{t-1}_g{g}")
                # réification: is_at_g <=> P[k,t-1] == g+1 (P est 1-based)
                model.Add(P[(k, t-1)] == g+1).OnlyEnforceIf(is_at_g)
                model.Add(P[(k, t-1)] != g+1).OnlyEnforceIf(is_at_g.Not())

                # options autorisées à t : rester au même garage (g) ou aller dans un dépôt
                allowed = [g] + depot_indices
                b_vars = []
                for j in allowed:
                    b = model.NewBoolVar(f"b_move_k{k}_t{t}_fromg{g}_toj{j}")
                    b_vars.append(b)
                    model.Add(P[(k, t)] == j+1).OnlyEnforceIf(b)
                    model.Add(P[(k, t)] != j+1).OnlyEnforceIf(b.Not())

                # si is_at_g alors exactement une option doit être vraie
                model.Add(sum(b_vars) == 1).OnlyEnforceIf(is_at_g)
                
    
    ##Contraintes modelisant que si un vehicule revient au garage il ne sort plus
    # indices (0-based) des garages
    garage_indices = [i for i, place in enumerate(places_list) if place[0] == "G"]

    for k in range(1, nvehicule+1):
        for t in range(2, T):  # on commence à 1 pour permettre un départ initial du garage
            for g in garage_indices:
                is_at_g = model.NewBoolVar(f"is_at_g_k{k}_t{t}_g{g}")
                # is_at_g <=> P[k,t] == g+1
                model.Add(P[(k, t)] == g+1).OnlyEnforceIf(is_at_g)
                model.Add(P[(k, t)] != g+1).OnlyEnforceIf(is_at_g.Not())
                # si is_at_g alors pour tout u>t, P[k,u] == g+1
                for u in range(t+1, T+1):
                    model.Add(P[(k, u)] == g+1).OnlyEnforceIf(is_at_g)
                    
                    
                    
    ##une contraint pour dire qu'un véhicule ne doit pas changer de garage s'il doit revenir dans un garage
    garage_indices = [i for i, place in enumerate(places_list) if place[0] == "G"]

    for k in range(1, nvehicule+1):
        # récupérer l'indice (0-based) du garage initial du véhicule k
        initial_garage_id = int(vehiculeFleet[k-1][2])
        initial_idx = next(i for i, place in enumerate(places_list) if place[0] == "G" and int(place[1]) == initial_garage_id)

        # Interdire d'être dans un garage différent du garage initial, pour tout t
        for t in range(0, T+1):
            for g in garage_indices:
                if g != initial_idx:
                    model.Add(P[(k, t)] != g+1)
    
    #Au garage le volume ne change pas
    for k in range(1, nvehicule+1):
        for t in range(1, T+1):
            for g in garage_indices:
                is_at_g = model.NewBoolVar(f"is_at_g_k{k}_t{t}_g{g}")
                model.Add(P[(k, t)] == g+1).OnlyEnforceIf(is_at_g)
                model.Add(P[(k, t)] != g+1).OnlyEnforceIf(is_at_g.Not())
                # Si le véhicule est au garage g à l'instant t, sa charge ne change pas entre t-1 et t
                model.Add(QCA[(k, t)] == QCA[(k, t-1)]).OnlyEnforceIf(is_at_g)
                
                
    ##A t=0 et t=T les véhicules sont dans les garages (déjà fait pour t=0)
    for k in range(1, nvehicule+1):
        # récupérer l'indice (0-based) du garage initial du véhicule k
        initial_garage_id = int(vehiculeFleet[k-1][2])
        initial_idx = next(i for i, place in enumerate(places_list) if place[0] == "G" and int(place[1]) == initial_garage_id)
        # contrainte pour t=T
        model.Add(P[(k, T)] == initial_idx + 1)
        
    ##Contraintes modelisant qu'un camion qui reste au garage a t=1 ne sort pas
    for k in range(1, nvehicule+1):
        # récupérer l'indice (0-based) du garage initial du véhicule k
        initial_garage_id = int(vehiculeFleet[k-1][2])
        initial_idx = next(i for i, place in enumerate(places_list) if place[0] == "G" and int(place[1]) == initial_garage_id)

        # Si P[k,1] == initial_idx + 1 alors pour tout t>1, P[k,t] == initial_idx + 1
        is_at_initial_garage_t1 = model.NewBoolVar(f"is_at_initial_garage_k{k}_t1")
        model.Add(P[(k, 1)] == initial_idx + 1).OnlyEnforceIf(is_at_initial_garage_t1)
        model.Add(P[(k, 1)] != initial_idx + 1).OnlyEnforceIf(is_at_initial_garage_t1.Not())

        for t in range(2, T+1):
            model.Add(P[(k, t)] == initial_idx + 1).OnlyEnforceIf(is_at_initial_garage_t1)
    
    
    
    
    # Pour chaque véhicule k et pas t, si à t-1 il était dans le dépôt d_idx,
    # alors à t il doit aller dans UNE station.
    depot_indices = [i for i, place in enumerate(places_list) if place[0] == "D"]
    station_indices = [i for i, place in enumerate(places_list) if place[0] == "S"]

    for k in range(1, nvehicule+1):
        for t in range(1, T):
            for d_idx in depot_indices:
                is_at_d_prev = model.NewBoolVar(f"is_at_d_k{k}_t{t}_d{d_idx}")
                model.Add(P[(k, t)] == d_idx+1).OnlyEnforceIf(is_at_d_prev)
                model.Add(P[(k, t)] != d_idx+1).OnlyEnforceIf(is_at_d_prev.Not())

                # booléens pour aller vers UNE station (interdit de rester au dépôt)
                b_stations = []
                for s_idx in station_indices:
                    b = model.NewBoolVar(f"b_toS_k{k}_t{t}_fromd{d_idx}_to{s_idx}")
                    b_stations.append(b)
                    model.Add(P[(k, t+1)] == s_idx+1).OnlyEnforceIf(b)
                    model.Add(P[(k, t+1)] != s_idx+1).OnlyEnforceIf(b.Not())

                # Si il était au dépôt à t, il doit aller vers exactement UNE station à t+1
                model.Add(sum(b_stations) == 1).OnlyEnforceIf(is_at_d_prev)



    # Si un véhicule k est au dépôt d à l'instant t et transporte le produit p,
    # alors sa charge augmente (strictement) entre t-1 et t et la baisse du dépôt
    # pour ce produit est exactement égale à cette augmentation.
    for k in range(1, nvehicule+1):
        for t in range(1, T):
            for d in range(1, ndepot+1):
                depot_id = depotMat[d-1][0]
                depot_index = next(i for i, place in enumerate(places_list) if place[0] == "D" and place[1] == depot_id)
                for p in range(1, nprod+1):
                    is_at_depot = model.NewBoolVar(f"is_at_depot_k{k}_t{t}_d{d}_p{p}")
                    is_prod = model.NewBoolVar(f"is_prod_k{k}_t{t}_p{p}")

                    model.Add(P[(k, t)] == depot_index + 1).OnlyEnforceIf(is_at_depot)
                    model.Add(P[(k, t)] != depot_index + 1).OnlyEnforceIf(is_at_depot.Not())

                    model.Add(TC[(k, t)] == p).OnlyEnforceIf(is_prod)
                    model.Add(TC[(k, t)] != p).OnlyEnforceIf(is_prod.Not())

                    # charge du camion augmente strictement
                    model.Add(QCA[(k, t)] > QCA[(k, t-1)]).OnlyEnforceIf([is_at_depot, is_prod])
                    # diminution du dépôt égale à l'augmentation du camion
                    model.Add(QCA[(k, t)] - QCA[(k, t-1)] == QD[(d, p, t-1)] - QD[(d, p, t)]).OnlyEnforceIf([is_at_depot, is_prod])


    # si aucun camion ne charge au dépôt pour un produit p entre t-1 et t, alors QD[(d,p,t)] = QD[(d,p,t-1)] ;
    # si le dépôt perd du volume, cette perte est exactement la somme des quantités chargées par les véhicules présents.
    
    LOAD = {}  # LOAD[(k,d,p,t)] = quantité chargée par k au dépôt d pour produit p entre t-1 et t

    for k in range(1, nvehicule+1):
        for d in range(1, ndepot+1):
            depot_id = depotMat[d-1][0]
            depot_index = next(i for i, place in enumerate(places_list) if place[0] == "D" and place[1] == depot_id)
            max_cap = int(vehiculeFleet[k-1][1])
            for p in range(1, nprod+1):
                for t in range(1, T+1):  # intervalle [t-1,t]
                    LOAD[(k,d,p,t)] = model.NewIntVar(0, max_cap, f"LOAD_{k}_{d}_{p}_{t}")

                    is_at_depot = model.NewBoolVar(f"is_at_depot_k{k}_t{t}_d{d}")
                    is_prod    = model.NewBoolVar(f"is_prod_k{k}_t{t}_p{p}")

                    # réifications
                    model.Add(P[(k,t)] == depot_index+1).OnlyEnforceIf(is_at_depot)
                    model.Add(P[(k,t)] != depot_index+1).OnlyEnforceIf(is_at_depot.Not())
                    model.Add(TC[(k,t)] == p).OnlyEnforceIf(is_prod)
                    model.Add(TC[(k,t)] != p).OnlyEnforceIf(is_prod.Not())

                    # si présent et même produit → LOAD = augmentation du camion
                    model.Add(LOAD[(k,d,p,t)] == QCA[(k,t)] - QCA[(k,t-1)]).OnlyEnforceIf([is_at_depot, is_prod])

                    # sinon pas de charge
                    model.Add(LOAD[(k,d,p,t)] == 0).OnlyEnforceIf(is_at_depot.Not())
                    model.Add(LOAD[(k,d,p,t)] == 0).OnlyEnforceIf(is_prod.Not())

    # Agrégation sur les véhicules : la baisse du dépôt = somme des LOAD
    for d in range(1, ndepot+1):
        for p in range(1, nprod+1):
            for t in range(1, T+1):
                model.Add(QD[(d, p, t-1)] - QD[(d, p, t)] == sum(LOAD[(k,d,p,t)] for k in range(1, nvehicule+1)))
                
                
    #Si un camion doit changer du type de produit à un depot donné son volume doit être vide
    for k in range(1, nvehicule+1):
        for t in range(1, T):
            for d in range(1, ndepot+1):
                depot_id = depotMat[d-1][0]
                depot_index = next(i for i, place in enumerate(places_list) if place[0] == "D" and place[1] == depot_id)

                is_at_depot_t = model.NewBoolVar(f"is_at_depot_k{k}_t{t}_d{d}")
                model.Add(P[(k, t)] == depot_index + 1).OnlyEnforceIf(is_at_depot_t)
                model.Add(P[(k, t)] != depot_index + 1).OnlyEnforceIf(is_at_depot_t.Not())

                change_prod = model.NewBoolVar(f"change_prod_k{k}_t{t}_d{d}")
                model.Add(TC[(k, t)] != TC[(k, t-1)]).OnlyEnforceIf(change_prod)
                model.Add(TC[(k, t)] == TC[(k, t-1)]).OnlyEnforceIf(change_prod.Not())

                # (optionnel) forcer que le changement n'arrive que si on est au dépôt
                model.Add(is_at_depot_t == 1).OnlyEnforceIf(change_prod)

                # si changement au dépôt alors le camion était vide avant (t-1)
                model.Add(QCA[(k, t-1)] == 0).OnlyEnforceIf([is_at_depot_t, change_prod])
            
    ##Si un camion arrive dans une station s au temps t transportant le produit p,
    ##alors la quantité du camion diminue strictement entre t-1 et t et la quantité de la station augmente exactement de cette diminution.
    # Variables de livraison et contraintes associées
    DEL = {}  # DEL[(k,s,p,t)] = quantité livrée par k à la station s pour produit p entre t-1 et t

    for k in range(1, nvehicule+1):
        max_cap = int(vehiculeFleet[k-1][1])
        for s in range(1, nstation+1):
            station_id = stationMat[s-1][0]
            station_index = next(i for i, place in enumerate(places_list) if place[0] == "S" and place[1] == station_id)
            for p in range(1, nprod+1):
                for t in range(1, T+1):
                    DEL[(k, s, p, t)] = model.NewIntVar(0, max_cap, f"DEL_{k}_{s}_{p}_{t}")

                    is_at_s = model.NewBoolVar(f"is_at_s_k{k}_t{t}_s{s}")
                    is_prod = model.NewBoolVar(f"is_prod_k{k}_t{t}_p{p}")

                    # réifications
                    model.Add(P[(k, t)] == station_index + 1).OnlyEnforceIf(is_at_s)
                    model.Add(P[(k, t)] != station_index + 1).OnlyEnforceIf(is_at_s.Not())
                    model.Add(TC[(k, t)] == p).OnlyEnforceIf(is_prod)
                    model.Add(TC[(k, t)] != p).OnlyEnforceIf(is_prod.Not())

                    # si le camion est à la station s à t et transporte p :
                    # - sa charge diminue strictement entre t-1 et t
                    # - la livraison DEL est égale à cette diminution
                    model.Add(QCA[(k, t)] < QCA[(k, t-1)]).OnlyEnforceIf([is_at_s, is_prod])
                    model.Add(DEL[(k, s, p, t)] == QCA[(k, t-1)] - QCA[(k, t)]).OnlyEnforceIf([is_at_s, is_prod])

                    # sinon DEL = 0
                    model.Add(DEL[(k, s, p, t)] == 0).OnlyEnforceIf(is_at_s.Not())
                    model.Add(DEL[(k, s, p, t)] == 0).OnlyEnforceIf(is_prod.Not())

    # Agrégation : l'augmentation de la station = somme des livraisons reçues
    for s in range(1, nstation+1):
        for p in range(1, nprod+1):
            for t in range(1, T+1):
                model.Add(QS[(s, p, t)] - QS[(s, p, t-1)] == sum(DEL[(k, s, p, t)] for k in range(1, nvehicule+1)))
                
                
    
    # Forcer un départ immédiat : si un camion k est à la station s au temps t,
    # il ne peut pas être à la même station au temps t+1.
    for k in range(1, nvehicule+1):
        for s in range(1, nstation+1):
            station_id = stationMat[s-1][0]
            station_index = next(i for i, place in enumerate(places_list) if place[0] == "S" and place[1] == station_id)
            for t in range(1, T):                   # t de 1 à T-1, on regarde t+1
                is_at_s = model.NewBoolVar(f"is_at_s_k{k}_t{t}_s{s}")
                model.Add(P[(k, t)] == station_index + 1).OnlyEnforceIf(is_at_s)
                model.Add(P[(k, t)] != station_index + 1).OnlyEnforceIf(is_at_s.Not())
                # si présent à t alors à t+1 il doit quitter la station
                model.Add(P[(k, t+1)] != station_index + 1).OnlyEnforceIf(is_at_s)
    
    
    
    # def add_objective_min_changeover_distance(model, P, TC, transtionCostMat, distanceMat,
    #                                         nprod, nbre_places_totales, nvehicule, T, scale=100):
    #     # conversion en entiers (préserve décimales via scale)
    #     cost_scaled = [[int(round(c * scale)) for c in row] for row in transtionCostMat]
    #     dist_scaled = [[int(round(d * scale)) for d in row] for row in distanceMat]

    #     total_terms = []
        
    #     for k in range(1, nvehicule+1):
    #         for t in range(1, T+1):
    #             b_trans = {}
    #             for p in range(1, nprod+1):
    #                 for q in range(1, nprod+1):
    #                     b = model.NewBoolVar(f"b_trans_k{k}_t{t}_p{p}_q{q}")
    #                     c = model.NewBoolVar(f"c")
    #                     d = model.NewBoolVar(f"d")
    #                     b_trans[(p, q)] = b
                        
    #                     model.Add(TC[(k, t-1)] == p).OnlyEnforceIf(c)
    #                     model.Add(TC[(k, t)] == q).OnlyEnforceIf(d)
                        
    #                     model.Add(TC[(k, t-1)] != p).OnlyEnforceIf(c.Not())
    #                     model.Add(TC[(k, t)] != q).OnlyEnforceIf(d.Not())
                        
    #                     model.Add(b == 1).OnlyEnforceIf([c, d])
    #             model.Add(sum(b_trans.values()) == 1)

    #             for (p, q), b in b_trans.items():
    #                 total_terms.append(cost_scaled[p-1][q-1] * b)

    #             # Distance parcourue: réifier (P_{t-1} -> P_t)
    #             b_pos = {}
    #             for i in range(1, nbre_places_totales+1):
    #                 for j in range(1, nbre_places_totales+1):
    #                     b = model.NewBoolVar(f"b_pos_k{k}_t{t}_i{i}_j{j}")
    #                     b_pos[(i, j)] = b
                        
    #                     c = model.NewBoolVar(f"c")
    #                     d = model.NewBoolVar(f"d")
                        
    #                     model.Add(P[(k, t-1)] == i).OnlyEnforceIf(c)
    #                     model.Add(P[(k, t-1)] != i).OnlyEnforceIf(c.Not())
    #                     model.Add(P[(k, t)] == j).OnlyEnforceIf(d)
    #                     model.Add(P[(k, t)] != j).OnlyEnforceIf(d.Not())
    #                     model.Add(b == 1).OnlyEnforceIf([c, d])
    #             model.Add(sum(b_pos.values()) == 1)
    #             for (i, j), b in b_pos.items():
    #                 total_terms.append(dist_scaled[i-1][j-1] * b)

    #     model.Minimize(sum(total_terms))

    # # Appel (ajuste scale si besoin, ex: 1000 pour 3 décimales)
    # add_objective_min_changeover_distance(model, P, TC, transtionCostMat, distanceMat,
    #                                     nprod, nbre_places_totales, nvehicule, T, scale=100)
    
    
    
    
    
    

    
    solver = cp_model.CpSolver()
    status = solver.Solve(model)
    print(f"Status: {solver.StatusName(status)}")
    
    # ###Afficher les positions des camions
    # if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    #     for k in range(1, nvehicule+1):
    #         print(f"Trajet du camion {k}:")
    #         for t in range(0, T+1):
    #             pos_index = solver.Value(P[(k,t)]) - 1
    #             pos = places_list[pos_index]
    #             prod_type = solver.Value(TC[(k,t)])
    #             quantity = solver.Value(QCA[(k,t)])
    #             print(f"  Temps {t}: Position {pos[0]}-{pos[1]} at ({pos[2][0]}, {pos[2][1]}), Produit Type: {prod_type}, Quantité: {quantity}")
    #         print()
    
    
    # ###Aficher la quantité des stations par produit et par temps
    # for s in range(1, nstation+1):  
    #     print(f"Stock de la station {s}:")
    #     for p in range(1, nprod+1):
    #         print(f"  Produit {p}: ", end="")
    #         for t in range(0, T+1):
    #             quantity = solver.Value(QS[(s, p, t)])
    #             print(f"T{t}:{quantity} ", end="")
    #         print()
    #     print()
    
    # #Afficher la quantité des dépôts par produit et par temps
    # for d in range(1, ndepot+1):  
    #     print(f"Stock du dépôt {d}:")
    #     for p in range(1, nprod+1):
    #         print(f"  Produit {p}: ", end="")
    #         for t in range(0, T+1):
    #             quantity = solver.Value(QD[(d, p, t)])
    #             print(f"T{t}:{quantity} ", end="")
    #         print()
    #     print()
    
    ###Affichage synthétique par camion : trajet + produit (coût change)

    
    
    # Affichage synthétique par camion : trajet + produit (coût change)
    to_return = ""
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        for k in range(1, nvehicule+1):
            #garage du camion
            garage_id = vehiculeFleet[k-1][2]
            garage_index = next(i for i, place in enumerate(places_list) if place[0] == "G" and int(place[1]) == garage_id)
            
            if solver.Value(P[(k, 1)]) - 1 == garage_index:
                #print(f"{k}: Le camion n'est pas sorti du garage.")
                continue
            route_parts = []
            prod_parts = []

            for t in range(0, T+1):
                pos_index = solver.Value(P[(k, t)]) - 1
                place = places_list[pos_index]      # [type, id, (x,y)]
                part = str(place[1])

                # Quantité prise/délivrée entre t-1 et t (si t>=1)
                if t >= 1 and t < T:
                    pos_0_index = solver.Value(P[(k, t+1)]) - 1
                    if pos_index == pos_0_index:
                        continue
                    delta = solver.Value(QCA[(k, t)]) - solver.Value(QCA[(k, t-1)])
                    # Si on est dans un dépôt et delta>0 => prise (afficher entre crochets)
                    if place[0] == "D" and delta > 0:
                        part += f" [{delta}]"
                    # Si on est dans une station et delta<0 => livraison (afficher entre parenthèses)
                    if place[0] == "S" and delta < 0:
                        part += f" ({-delta})"

                route_parts.append(part)

                # Produit (0-based) et coût de change entre t-1 et t
                prod = solver.Value(TC[(k, t)]) - 1
                if t == 0:
                    cost = 0.0
                else:
                    prev = solver.Value(TC[(k, t-1)]) - 1
                    cost = transtionCostMat[prev][prod]
                prod_parts.append(f"{prod}({cost:.1f})")

            to_return += f"{k}: " + " - ".join(route_parts) + "\n"
            to_return += f"{k}: " + " - ".join(prod_parts) + "\n"
            to_return += "\n"
        
    ##calculer nombre de vehicules utilisés
    used_vehicles = 0
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        for k in range(1, nvehicule+1):
            garage_id = vehiculeFleet[k-1][2]
            garage_index = next(i for i, place in enumerate(places_list) if place[0] == "G" and int(place[1]) == garage_id)
            if solver.Value(P[(k, 1)]) - 1 != garage_index:
                used_vehicles += 1
    to_return += f"{used_vehicles}\n"
    
    ##calculer le nombre de changements de produit
    total_changeovers = 0
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        for k in range(1, nvehicule+1):
            for t in range(1, T+1):
                prod = solver.Value(TC[(k, t)]) - 1
                prod_prev = solver.Value(TC[(k, t-1)]) - 1
                if prod != prod_prev:
                    total_changeovers += 1
    to_return += f"{total_changeovers}\n"
    
    
    ##calculer le coût total de changement de produit
    total_changeover_cost = 0.0
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        for k in range(1, nvehicule+1):
            for t in range(1, T+1):
                prod = solver.Value(TC[(k, t)]) - 1
                prod_prev = solver.Value(TC[(k, t-1)]) - 1
                cost = transtionCostMat[prod_prev][prod]
                total_changeover_cost += cost
    to_return += f"{total_changeover_cost:.2f}\n"
    
    
    ##calculer la distance totale parcourue
    total_distance = 0.0
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        for k in range(1, nvehicule+1):
            for t in range(1, T+1):
                pos_index = solver.Value(P[(k, t)]) - 1
                pos_prev_index = solver.Value(P[(k, t-1)]) - 1
                dist = distanceMat[pos_prev_index][pos_index]
                total_distance += dist
    to_return += f"{total_distance:.2f}\n"
    to_return += "Intel® Core™ i7-10850H × 12"
    return to_return

# nprod = 2
# ndepot= 1
# ngarage = 2
# transtionCostMat = [[0, 18.1], [61.5, 0]]
# vehiculeFleet = [["1", 20000, 1, 1], ["2", 20000, 1, 2]]
# nstation = 3
# nvehicule = 2
# depotMat = [["1", 81.6, 63.6, 57914, 82626]]
# garageMat = [["1", 98.1, 49.6], ["2", 56.8, 26]]
# stationMat = [["1", 23.5, 42.2, 0, 4278], ["2", 3.5, 38.3, 1344, 2350], ["3", 56.7, 31.0, 0, 2319]]

# result = modelsRo(nprod, ndepot, ngarage, nstation, nvehicule, transtionCostMat, vehiculeFleet, depotMat, garageMat, stationMat)
# print(result)


def extraire_inf(file_name="small/MPVRP_S_001_s9_d1_p2.dat"):
    with open(file_name, 'r') as file:
        lines = file.readlines()

    nprod = int(lines[1].strip().split()[0])
    ndepot = int(lines[1].strip().split()[1])
    ngarage = int(lines[1].strip().split()[2])
    nstation = int(lines[1].strip().split()[3])
    nvehicule = int(lines[1].strip().split()[4])
    
    
    transtionCostMat = []
    for i in range(2, 2 + nprod):
        row = list(map(float, lines[i].strip().split()))
        transtionCostMat.append(row)
    
    # nvehicule = int(lines[3 + nprod].strip())
    vehiculeFleet = []
    for i in range(2 + nprod, 2 + nprod + nvehicule):
        parts = lines[i].strip().split()
        vehiculeFleet.append([parts[0], int(parts[1]), int(parts[2]), int(parts[3])])
    
    # ndepot_line = 4 + nprod + nvehicule
    depotMat = []
    for i in range(2 + nprod + nvehicule, 2 + nprod + nvehicule + ndepot):
        parts = lines[i].strip().split()
        depot_id = parts[0]
        x = float(parts[1])
        y = float(parts[2])
        stocks = list(map(int, parts[3:3 + nprod]))
        depotMat.append([depot_id, x, y] + stocks)
    
    # ngarage_line = ndepot_line + ndepot
    garageMat = []
    for i in range(2 + nprod + nvehicule + ndepot, 2 + nprod + nvehicule + ndepot + ngarage):
        parts = lines[i].strip().split()
        garage_id = parts[0]
        x = float(parts[1])
        y = float(parts[2])
        garageMat.append([garage_id, x, y])
    
    # nstation_line = ngarage_line + ngarage
    # nstation = int(lines[nstation_line].strip())
    stationMat = []
    for i in range(2 + nprod + nvehicule + ndepot + ngarage, 2 + nprod + nvehicule + ndepot + ngarage + nstation):
        parts = lines[i].strip().split()
        station_id = parts[0]
        x = float(parts[1])
        y = float(parts[2])
        demands = list(map(int, parts[3:3 + nprod]))
        stationMat.append([station_id, x, y] + demands)
    
    # print(f"nprod: {nprod}, ndepot: {ndepot}, ngarage: {ngarage}, nstation: {nstation}, nvehicule: {nvehicule}, transtionCostMat: {transtionCostMat}, vehiculeFleet: {vehiculeFleet}, depotMat: {depotMat},garageMat: {garageMat}, stationMat: {stationMat}")
    return (nprod, ndepot, ngarage, nstation, nvehicule, transtionCostMat, vehiculeFleet, depotMat, garageMat, stationMat, file_name.split("/")[-1])

#modelsRo(*extraire_inf("small/MPVRP_S_001_s9_d1_p2.dat"))

def write_output(nprod, ndepot, ngarage, nstation, nvehicule, transtionCostMat, vehiculeFleet, depotMat, garageMat, stationMat,file_name=""):
    import time
    name = file_name.split("/")[-1]
    file_name = f"Sol_{name}"
    a = time.time()
    content = modelsRo(nprod, ndepot, ngarage, nstation, nvehicule, transtionCostMat, vehiculeFleet, depotMat, garageMat, stationMat)
    b = time.time()
    content += f"\n{b - a:.2f}\n"
    with open(file_name, 'w') as file:
        file.write(content)
        
write_output(*extraire_inf("small/MPVRP_S_033_s13_d1_p2.dat"))
        

