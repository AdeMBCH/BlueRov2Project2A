#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Nom du fichier CSV
csv_filename = "../../../yolo_results.csv"

# On initialise les compteurs
TP_total, FP_total, FN_total, TN_total = 0, 0, 0, 0

# Lecture du CSV et sommation
with open(csv_filename, "r", newline="") as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        try:
            tp = int(row["TP"]) if row["TP"].strip() else 0
            fp = int(row["FP"]) if row["FP"].strip() else 0
            fn = int(row["FN"]) if row["FN"].strip() else 0
            tn = int(row["TN"]) if row["TN"].strip() else 0
        except (KeyError, ValueError):
            # Si la colonne n'existe pas ou la valeur est vide/invalide
            tp, fp, fn, tn = 0, 0, 0, 0

        TP_total += tp
        FP_total += fp
        FN_total += fn
        TN_total += tn

# Calcul des métriques
precision = TP_total / (TP_total + FP_total) if (TP_total + FP_total) > 0 else 0
recall    = TP_total / (TP_total + FN_total) if (TP_total + FN_total) > 0 else 0
f1_score  = (2.0 * precision * recall) / (precision + recall) if (precision + recall) > 0 else 0

# Création de la figure et de l'axe
fig, ax = plt.subplots(figsize=(6, 6))

# Retrait des graduations et des bordures
ax.set_xticks([])
ax.set_yticks([])
for spine in ax.spines.values():
    spine.set_visible(False)

# Dessin de la matrice de confusion
# La matrice 2×2 s'étend de x=0 à x=2, et y=0 à y=2
#   - (0,1) => TP
tp_rect = patches.Rectangle((0,1), 1, 1, fill=True, color="limegreen", alpha=0.6)
ax.add_patch(tp_rect)
ax.text(0.5, 1.5, f"TP = {TP_total}", ha="center", va="center", fontsize=12, fontweight="bold")

#   - (1,1) => FP
fp_rect = patches.Rectangle((1,1), 1, 1, fill=True, color="red", alpha=0.4)
ax.add_patch(fp_rect)
ax.text(1.5, 1.5, f"FP = {FP_total}", ha="center", va="center", fontsize=12, fontweight="bold")

#   - (0,0) => FN
fn_rect = patches.Rectangle((0,0), 1, 1, fill=True, color="orange", alpha=0.4)
ax.add_patch(fn_rect)
ax.text(0.5, 0.5, f"FN = {FN_total}", ha="center", va="center", fontsize=12, fontweight="bold")

#   - (1,0) => TN
tn_rect = patches.Rectangle((1,0), 1, 1, fill=True, color="lightgray", alpha=0.6)
ax.add_patch(tn_rect)
ax.text(1.5, 0.5, f"TN = {TN_total}", ha="center", va="center", fontsize=12, fontweight="bold")

# On règle les limites d'affichage
ax.set_xlim(0, 2)
ax.set_ylim(0, 2)

# Étiquettes (Réel, Prédit)
ax.text(0.5, 2.1, "Réel : Positif", ha="center", va="center", fontsize=12)
ax.text(1.5, 2.1, "Réel : Négatif", ha="center", va="center", fontsize=12)
ax.text(-0.1, 1.5, "Prédit : Positif", ha="center", va="center",
        rotation=90, fontsize=12)
ax.text(-0.1, 0.5, "Prédit : Négatif", ha="center", va="center",
        rotation=90, fontsize=12)

# On réduit un peu la partie utilisée pour laisser la place à droite
plt.subplots_adjust(right=0.7)

# Texte contenant les métriques
metrics_text = (
    f"Précision : {precision:.2f}\n"
    f"Rappel    : {recall:.2f}\n"
    f"F1-score  : {f1_score:.2f}"
)

# Affichage du texte sur la droite (75% de la largeur, milieu en hauteur)
fig.text(
    0.75, 0.5,  # x=0.75 (à droite), y=0.5 (milieu vertical)
    metrics_text,
    ha='left',
    va='center',
    fontsize=12
)

plt.show()
