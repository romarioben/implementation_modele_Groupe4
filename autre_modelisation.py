def add_objective_min_changeover_distance(model, P, TC, transtionCostMat, distanceMat,
                                            nprod, nbre_places_totales, nvehicule, T, scale=100):
        # conversion en entiers (préserve décimales via scale)
        cost_scaled = [[int(round(c * scale)) for c in row] for row in transtionCostMat]
        dist_scaled = [[int(round(d * scale)) for d in row] for row in distanceMat]

        total_terms = []

        for k in range(1, nvehicule+1):
            for t in range(1, T+1):
                # Coût de changeover: réifier (TC_{t-1} -> TC_t)
                b_trans = {}
                for p in range(1, nprod+1):
                    for q in range(1, nprod+1):
                        b = model.NewBoolVar(f"b_trans_k{k}_t{t}_p{p}_q{q}")
                        b_trans[(p, q)] = b
                        model.Add(TC[(k, t-1)] == p).OnlyEnforceIf(b)
                        model.Add(TC[(k, t-1)] != p).OnlyEnforceIf(b.Not())
                        model.Add(TC[(k, t)] == q).OnlyEnforceIf(b)
                        model.Add(TC[(k, t)] != q).OnlyEnforceIf(b.Not())
                model.Add(sum(b_trans.values()) == 1)
                for (p, q), b in b_trans.items():
                    total_terms.append(cost_scaled[p-1][q-1] * b)

                # Distance parcourue: réifier (P_{t-1} -> P_t)
                b_pos = {}
                for i in range(1, nbre_places_totales+1):
                    for j in range(1, nbre_places_totales+1):
                        b = model.NewBoolVar(f"b_pos_k{k}_t{t}_i{i}_j{j}")
                        b_pos[(i, j)] = b
                        model.Add(P[(k, t-1)] == i).OnlyEnforceIf(b)
                        model.Add(P[(k, t-1)] != i).OnlyEnforceIf(b.Not())
                        model.Add(P[(k, t)] == j).OnlyEnforceIf(b)
                        model.Add(P[(k, t)] != j).OnlyEnforceIf(b.Not())
                model.Add(sum(b_pos.values()) == 1)
                for (i, j), b in b_pos.items():
                    total_terms.append(dist_scaled[i-1][j-1] * b)

        model.Minimize(sum(total_terms))

    # Appel (ajuste scale si besoin, ex: 1000 pour 3 décimales)
    add_objective_min_changeover_distance(model, P, TC, transtionCostMat, distanceMat,
                                        nprod, nbre_places_totales, nvehicule, T, scale=100)
    
    