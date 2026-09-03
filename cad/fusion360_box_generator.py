# ============================================================
#  Generatore parametrico box COLLARE + PALMARE per Fusion 360
# ============================================================
#
#  COME USARLO:
#   1. Apri Fusion 360, crea un NUOVO DESIGN (File > New Design, NON New
#      Part: un Part e' a componente singolo e questo script ne crea due).
#   2. Utilities > Scripts and Add-Ins (o Shift+S) > scheda "Scripts"
#      > "+" (Create) > "Python" > dagli un nome (es. BoxGenerator)
#      > si apre l'editor: incolla dentro tutto questo file, salva.
#   3. Prima di lanciarlo: aggiorna le costanti nella sezione PARAMETRI
#      qui sotto con le misure REALI prese col calibro sui tuoi
#      componenti (scheda, batteria, GPS, connettori antenna, ecc). I
#      valori messi ora sono placeholder plausibili ma NON misurati - se
#      li lanci cosi' come sono il modello sara' solo indicativo.
#   4. Esegui (Run). Vengono creati due componenti nel documento:
#      "Collare" e "Palmare", ognuno con due solidi: "Base" e "Lid".
#
#  COSA GENERA (per ciascun dispositivo):
#   - Base: guscio con pareti (shell), spigoli verticali arrotondati,
#     4 colonnette con foro pilota agli angoli per le viti di chiusura,
#     sollevate dal fondo per far entrare la scheda sopra.
#   - Lid: coperchio piatto che chiude la base dall'alto, con i 4 fori
#     passanti allineati alle colonnette.
#   - Collare (case IP65, solo il "baccello" nero - NON tutta la fascia
#     del collare, quella resta il cinturino esistente del cane):
#       - canalina per guarnizione sul bordo superiore della Base, sotto
#         il Lid, per la tenuta stagna (vedi add_gasket_channel).
#       - foro sul Lid per il passacavo/connettore stagno dell'antenna
#         LoRa (va integrato un pressacavo o connettore SMA con O-ring,
#         il solo foro nella stampa NON e' impermeabile da solo).
#       - due "alette" solide sui lati corti con un'asola ciascuna, per
#         far passare la fascia del collare - FUORI dal volume stagno,
#         non lo intaccano (vedi add_mount_ears).
#       - asola USB-C e punto assottigliato per il reed switch, come sul
#         Palmare (vedi sotto).
#   - Palmare (stile radio ricetrasmittente, non testato per IP65 - se ti
#     serve anche li' la tenuta, la canalina si aggiunge allo stesso modo
#     del Collare, vedi add_gasket_channel):
#       - proporzioni alte e strette (impugnabile in verticale, non piatto
#         come prima).
#       - finestra per il display OLED su una parete laterale ("front"),
#         non piu' sul Lid.
#       - sul Lid: foro encoder + due fori antenna (GPS e LoRa) vicini tra
#         loro, in alto come richiesto - stesso discorso del connettore
#         stagno per le antenne, se serve tenuta anche qui.
#       - asola USB-C laterale e punto assottigliato per il reed switch
#         (SOLO se la tua versione del palmare ne ha uno fisico - il
#         firmware attuale usa il click/pressione dell'encoder per
#         accendere/spegnere, non un magnete: se non ti serve, non
#         chiamare thin_wall_spot() per il Palmare).
#
#  ATTENZIONE - taglio nella direzione sbagliata:
#   Tutti i tagli sulle pareti laterali/canalina/alette sono fatti su
#   piani costruiti "a occhio" nella direzione giusta, ma non ho potuto
#   testarli dentro Fusion (non ho accesso diretto al programma). Se dopo
#   l'esecuzione un taglio esce FUORI dal pezzo invece che scavarci
#   dentro, e' quasi certamente un problema di segno: cerca il parametro
#   "depth"/"distance" della chiamata corrispondente in fondo al file e
#   prova a invertirlo (positivo <-> negativo). Il resto (Base, Lid,
#   colonnette, arrotondamenti) segue pattern standard della API e
#   dovrebbe funzionare cosi' com'e'.
#
# ============================================================

import adsk.core, adsk.fusion, adsk.cam, traceback


def mm(v):
    """Fusion lavora in cm internamente: converte millimetri -> cm."""
    return v / 10.0


# ============================================================
#  PARAMETRI — VERIFICA COL CALIBRO PRIMA DI LANCIARE
#  Tutte le misure in millimetri.
# ============================================================

# ---- Generali, condivisi da entrambi i box -------------------
WALL_T        = 2.0   # spessore pareti (stampa 3D FDM: 1.6-2.4mm tipico)
CORNER_R      = 3.0   # raggio arrotondamento spigoli verticali esterni
LID_T         = 2.0   # spessore del coperchio
BOSS_INSET    = 5.0   # distanza delle colonnette dai due bordi (X e Y)
BOSS_D        = 6.0   # diametro esterno colonnetta
BOSS_H_MARGIN = 1.0   # le colonnette arrivano fino a (H - questo margine)
SCREW_HOLE_D  = 2.6   # foro pilota per vite autofilettante M3 (o M2.5=2.0)

USB_SLOT_W    = 10.0  # larghezza asola USB-C
USB_SLOT_H    = 4.0   # altezza asola USB-C
USB_SLOT_Z    = 6.0   # altezza dal fondo del centro dell'asola

REED_SPOT_D      = 8.0   # diametro zona assottigliata per il magnete
REED_REMAINING_T = 1.6   # spessore di parete che deve restare (1.5-2mm da nota progetto)
REED_SPOT_Z      = 10.0  # altezza dal fondo del centro della zona

# Canalina guarnizione (tenuta IP65): un cordolo in gomma/silicone (o TPU
# morbido) alloggiato qui, compresso dal Lid avvitato. GASKET_GROOVE_W va
# tarato sul diametro reale del cordolo che userai (di solito leggermente
# piu' STRETTO del cordolo, cosi' viene compresso e non solo appoggiato).
GASKET_GROOVE_W     = 1.6   # larghezza canalina - cordolo tipico 2mm compresso
GASKET_GROOVE_DEPTH = 1.2   # profondita' canalina

# Foro per i pigtail SMA a pannello con dado/rondella che hai gia' - 6.5mm
# e' la misura standard per la filettatura SMA (1/4-36 UNS), verifica col
# calibro sul dado del TUO pigtail prima di stampare (le rondelle variano
# leggermente tra produttori). Questi connettori hanno anche uno spessore
# di pannello consigliato dal produttore (spesso 1.5-3mm) entro cui il
# dado stringe bene: il nostro WALL_T (2mm) di solito rientra, ma controlla
# la scheda tecnica del tuo pigtail per esserne sicuro - se il pannello e'
# troppo spesso il dado non arriva a stringere a fondo, se troppo sottile
# il connettore resta lasco.
ANTENNA_HOLE_D = 6.5

# ---- COLLARE: solo il "baccello" nero, non tutta la fascia ---
COLLARE_L = 55.0   # lunghezza interna utile: scheda + GPS + margini -- DA VERIFICARE
COLLARE_W = 32.0
COLLARE_H = 18.0   # scheda + batteria impilate o affiancate -- DA VERIFICARE

COLLARE_ANT_CX = 0.0   # posizione foro antenna LoRa sul Lid
COLLARE_ANT_CY = 10.0

# Alette di fissaggio alla fascia del collare (sui due lati corti, X=+-L/2)
EAR_W    = 14.0   # larghezza aletta (lungo Y)
EAR_T    = 4.0    # spessore aletta (sporge in X dal case)
EAR_H    = 10.0   # altezza aletta (lungo Z)
EAR_Z    = 2.0    # altezza da terra della base dell'aletta
STRAP_SLOT_W = 22.0  # larghezza asola per la fascia - LARGHEZZA DEL TUO COLLARE, verifica
STRAP_SLOT_H = 3.0   # altezza asola - spessore della fascia + un po' di gioco

# ---- PALMARE: stile radio ricetrasmittente, alto e stretto ---
PALMARE_L = 58.0    # larghezza (impugnatura)
PALMARE_W = 26.0    # spessore
PALMARE_H = 125.0   # altezza - TUTTO DA VERIFICARE su scheda/batteria/display reali

# Finestra display, su parete "front" (Y = -W/2)
PALMARE_DISPLAY_W = 24.0  # -- MISURA IL TUO MODULO OLED
PALMARE_DISPLAY_H = 24.0
PALMARE_DISPLAY_Z = 80.0  # altezza da terra del centro finestra

# Sul Lid, in alto: encoder + 2 antenne (GPS e LoRa) vicine tra loro
PALMARE_ENC_HOLE_D = 7.2   # diametro albero encoder + gioco -- VERIFICA IL TUO ENCODER
PALMARE_ENC_CX = 0.0
PALMARE_ENC_CY = -8.0

PALMARE_ANT_LORA_CX = -14.0
PALMARE_ANT_LORA_CY = 6.0
PALMARE_ANT_GPS_CX  = 14.0
PALMARE_ANT_GPS_CY  = 6.0


# ============================================================
#  HELPER GEOMETRICI RIUTILIZZABILI
# ============================================================

def new_component(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def rect_profile_on_plane(comp, plane, cx, cy, w, h):
    """Disegna un rettangolo w x h centrato su (cx,cy) sul piano dato, ritorna il profilo."""
    sketch = comp.sketches.add(plane)
    lines = sketch.sketchCurves.sketchLines
    p0 = adsk.core.Point3D.create(mm(cx - w / 2), mm(cy - h / 2), 0)
    p1 = adsk.core.Point3D.create(mm(cx + w / 2), mm(cy + h / 2), 0)
    lines.addTwoPointRectangle(p0, p1)
    return sketch.profiles.item(0), sketch


def _fillet_vertical_edges(comp, body, corner_r):
    if corner_r <= 0:
        return
    vertical_edges = adsk.core.ObjectCollection.create()
    for edge in body.edges:
        geo = edge.geometry
        if geo.curveType == adsk.core.Curve3DTypes.Line3DCurveType:
            line = adsk.core.Line3D.cast(geo)
            sp, ep = line.startPoint, line.endPoint
            dx, dy, dz = abs(sp.x - ep.x), abs(sp.y - ep.y), abs(sp.z - ep.z)
            if dx < 1e-6 and dy < 1e-6 and dz > 1e-6:
                vertical_edges.add(edge)
    if vertical_edges.count > 0:
        fillets = comp.features.filletFeatures
        filletInput = fillets.createInput()
        filletInput.addConstantRadiusEdgeSet(
            vertical_edges, adsk.core.ValueInput.createByReal(mm(corner_r)), True)
        fillets.add(filletInput)


def create_box_body(comp, name, L, W, H, corner_r):
    """Solido pieno L x W x H (base a z=0), spigoli verticali arrotondati."""
    prof, _ = rect_profile_on_plane(comp, comp.xYConstructionPlane, 0, 0, L, W)

    extrudes = comp.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(H)))
    ext = extrudes.add(extInput)
    body = ext.bodies.item(0)
    body.name = name

    _fillet_vertical_edges(comp, body, corner_r)
    return body


def shell_open_top(comp, body, wall_t):
    """Svuota il solido lasciando le pareti (wall_t) e il fondo, apre la faccia in alto."""
    top_face, top_z = None, -1e9
    for face in body.faces:
        z = face.boundingBox.maxPoint.z
        if z > top_z:
            top_z, top_face = z, face

    faces = adsk.core.ObjectCollection.create()
    faces.add(top_face)
    shells = comp.features.shellFeatures
    shellInput = shells.createInput(faces)
    shellInput.insideThickness = adsk.core.ValueInput.createByReal(mm(wall_t))
    shells.add(shellInput)


def add_corner_bosses(comp, body, L, W, wall_t, inset, boss_d, hole_d, boss_h):
    """4 colonnette con foro pilota, dal fondo interno (z=wall_t) fino a boss_h."""
    planes = comp.constructionPlanes
    planeInput = planes.createInput()
    planeInput.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(wall_t)))
    basePlane = planes.add(planeInput)

    cx = L / 2 - inset
    cy = W / 2 - inset
    centers = [(cx, cy), (-cx, cy), (-cx, -cy), (cx, -cy)]

    # colonnette (join)
    sketch = comp.sketches.add(basePlane)
    circles = sketch.sketchCurves.sketchCircles
    for x, y in centers:
        circles.addByCenterRadius(adsk.core.Point3D.create(mm(x), mm(y), 0), mm(boss_d) / 2)
    profs = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        profs.add(sketch.profiles.item(i))

    extrudes = comp.features.extrudeFeatures
    extInput = extrudes.createInput(profs, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    extInput.participantBodies = [body]
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(boss_h - wall_t)))
    extrudes.add(extInput)

    # fori pilota (cut, passano anche oltre per sicurezza)
    sketch2 = comp.sketches.add(basePlane)
    circles2 = sketch2.sketchCurves.sketchCircles
    for x, y in centers:
        circles2.addByCenterRadius(adsk.core.Point3D.create(mm(x), mm(y), 0), mm(hole_d) / 2)
    profs2 = adsk.core.ObjectCollection.create()
    for i in range(sketch2.profiles.count):
        profs2.add(sketch2.profiles.item(i))

    extInput2 = extrudes.createInput(profs2, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput2.participantBodies = [body]
    extInput2.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(boss_h - wall_t + 2)))
    extrudes.add(extInput2)

    return centers  # riusate per allineare i fori del Lid


def add_gasket_channel(comp, body, L, W, wall_t, base_h, groove_w, groove_depth):
    """Canalina per guarnizione (tenuta IP65), incisa sul bordo superiore
    della Base (nello spessore della parete), centrata nello spessore.
    Taglio a forma di "cornice" (due rettangoli concentrici nella stessa
    sketch, si usa solo la regione tra i due)."""
    outer_inset = max(0.2, (wall_t - groove_w) / 2)
    outer_l, outer_w = L / 2 - outer_inset, W / 2 - outer_inset
    inner_l, inner_w = outer_l - groove_w, outer_w - groove_w

    planes = comp.constructionPlanes
    planeInput = planes.createInput()
    planeInput.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(base_h)))
    topPlane = planes.add(planeInput)

    sketch = comp.sketches.add(topPlane)
    lines = sketch.sketchCurves.sketchLines
    lines.addTwoPointRectangle(
        adsk.core.Point3D.create(mm(-outer_l), mm(-outer_w), 0),
        adsk.core.Point3D.create(mm(outer_l), mm(outer_w), 0))
    lines.addTwoPointRectangle(
        adsk.core.Point3D.create(mm(-inner_l), mm(-inner_w), 0),
        adsk.core.Point3D.create(mm(inner_l), mm(inner_w), 0))

    # la sketch genera 2 profili: il rettangolo interno pieno (1 loop) e la
    # "cornice" tra i due rettangoli (2 loop, quella che vogliamo).
    frame_prof = None
    for i in range(sketch.profiles.count):
        p = sketch.profiles.item(i)
        if p.profileLoops.count == 2:
            frame_prof = p
            break
    if frame_prof is None:
        return  # geometria non valida (outer troppo vicino a inner) - salta

    extrudes = comp.features.extrudeFeatures
    extInput = extrudes.createInput(frame_prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput.participantBodies = [body]
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-mm(groove_depth)))
    extrudes.add(extInput)


def add_mount_ears(comp, body, L, ear_w, ear_t, ear_z, ear_h, slot_w, slot_h):
    """Due alette solide sui lati corti (X=+-L/2), ciascuna con un'asola
    per far passare la fascia del collare - fuori dal volume stagno."""
    extrudes = comp.features.extrudeFeatures
    planes = comp.constructionPlanes

    for side in ("left", "right"):
        x_off = -L / 2 if side == "left" else L / 2
        sign = -1 if side == "left" else 1

        planeInput = planes.createInput()
        planeInput.setByOffset(comp.yZConstructionPlane, adsk.core.ValueInput.createByReal(mm(x_off)))
        wallPlane = planes.add(planeInput)

        # aletta: rettangolo nel piano locale (asse orizzontale = Y globale, verticale = Z globale)
        sketch = comp.sketches.add(wallPlane)
        lines = sketch.sketchCurves.sketchLines
        lines.addTwoPointRectangle(
            adsk.core.Point3D.create(mm(-ear_w / 2), mm(ear_z), 0),
            adsk.core.Point3D.create(mm(ear_w / 2), mm(ear_z + ear_h), 0))
        prof = sketch.profiles.item(0)

        extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        extInput.participantBodies = [body]
        extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(sign * mm(ear_t)))
        extrudes.add(extInput)

        # asola nell'aletta per la fascia (taglio attraverso lo spessore dell'aletta)
        sketch2 = comp.sketches.add(wallPlane)
        lines2 = sketch2.sketchCurves.sketchLines
        slot_cz = ear_z + ear_h / 2
        lines2.addTwoPointRectangle(
            adsk.core.Point3D.create(mm(-slot_w / 2), mm(slot_cz - slot_h / 2), 0),
            adsk.core.Point3D.create(mm(slot_w / 2), mm(slot_cz + slot_h / 2), 0))
        prof2 = sketch2.profiles.item(0)

        extInput2 = extrudes.createInput(prof2, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput2.participantBodies = [body]
        extInput2.setDistanceExtent(False, adsk.core.ValueInput.createByReal(sign * mm(ear_t + 1)))
        extrudes.add(extInput2)


def create_lid(comp, name, L, W, lid_t, corner_r, base_h, hole_centers, hole_d):
    """Coperchio piatto che chiude la Base dall'alto (parte da z=base_h)."""
    planes = comp.constructionPlanes
    planeInput = planes.createInput()
    planeInput.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(mm(base_h)))
    lidPlane = planes.add(planeInput)

    prof, _ = rect_profile_on_plane(comp, lidPlane, 0, 0, L, W)
    extrudes = comp.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(lid_t)))
    ext = extrudes.add(extInput)
    body = ext.bodies.item(0)
    body.name = name

    _fillet_vertical_edges(comp, body, corner_r)

    # fori passanti allineati alle colonnette della base
    sketch2 = comp.sketches.add(lidPlane)
    circles2 = sketch2.sketchCurves.sketchCircles
    for x, y in hole_centers:
        circles2.addByCenterRadius(adsk.core.Point3D.create(mm(x), mm(y), 0), mm(hole_d) / 2)
    profs2 = adsk.core.ObjectCollection.create()
    for i in range(sketch2.profiles.count):
        profs2.add(sketch2.profiles.item(i))
    extInput2 = extrudes.createInput(profs2, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput2.participantBodies = [body]
    extInput2.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(lid_t + 1)))
    extrudes.add(extInput2)

    return body, lidPlane


def cut_lid_holes(comp, lid_body, lid_plane, lid_t, holes):
    """Fori tondi passanti sul Lid. 'holes' e' una lista di (cx, cy, diametro)
    - usala per encoder + antenne (GPS/LoRa) raggruppati in alto."""
    sketch = comp.sketches.add(lid_plane)
    circles = sketch.sketchCurves.sketchCircles
    for cx, cy, d in holes:
        circles.addByCenterRadius(adsk.core.Point3D.create(mm(cx), mm(cy), 0), mm(d) / 2)

    profs = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        profs.add(sketch.profiles.item(i))

    extrudes = comp.features.extrudeFeatures
    extInput = extrudes.createInput(profs, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput.participantBodies = [lid_body]
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(mm(lid_t + 1)))
    extrudes.add(extInput)


def cut_side_slot(comp, body, side, L, W, H, slot_w, slot_h, slot_z, depth):
    """Asola rettangolare su una parete laterale (side: 'front'=-Y, 'back'=+Y,
    'left'=-X, 'right'=+X). Taglia verso l'interno del pezzo.
    NOTA: direzione non testata in Fusion, vedi avviso in cima al file."""
    planes = comp.constructionPlanes
    planeInput = planes.createInput()

    if side in ("front", "back"):
        base = comp.xZConstructionPlane
        offset = -W / 2 if side == "front" else W / 2
        local_x = 0  # sul piano XZ l'asse orizzontale locale e' X
    else:
        base = comp.yZConstructionPlane
        offset = -L / 2 if side == "left" else L / 2
        local_x = 0  # sul piano YZ l'asse orizzontale locale e' Y

    planeInput.setByOffset(base, adsk.core.ValueInput.createByReal(mm(offset)))
    wallPlane = planes.add(planeInput)

    sketch = comp.sketches.add(wallPlane)
    lines = sketch.sketchCurves.sketchLines
    p0 = adsk.core.Point3D.create(mm(local_x - slot_w / 2), mm(slot_z - slot_h / 2), 0)
    p1 = adsk.core.Point3D.create(mm(local_x + slot_w / 2), mm(slot_z + slot_h / 2), 0)
    lines.addTwoPointRectangle(p0, p1)
    prof = sketch.profiles.item(0)

    extrudes = comp.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput.participantBodies = [body]
    # segno negativo: taglia verso l'interno del pezzo (vedi avviso in cima al file)
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-mm(depth)))
    extrudes.add(extInput)


def thin_wall_spot(comp, body, side, L, W, remaining_t, spot_d, spot_z, wall_t):
    """Punto assottigliato (taglio NON passante) per il reed switch/calamita.
    Lascia 'remaining_t' mm di parete nel punto indicato."""
    planes = comp.constructionPlanes
    planeInput = planes.createInput()

    if side in ("front", "back"):
        base = comp.xZConstructionPlane
        offset = -W / 2 if side == "front" else W / 2
    else:
        base = comp.yZConstructionPlane
        offset = -L / 2 if side == "left" else L / 2

    planeInput.setByOffset(base, adsk.core.ValueInput.createByReal(mm(offset)))
    wallPlane = planes.add(planeInput)

    sketch = comp.sketches.add(wallPlane)
    circles = sketch.sketchCurves.sketchCircles
    circles.addByCenterRadius(adsk.core.Point3D.create(0, mm(spot_z), 0), mm(spot_d) / 2)
    prof = sketch.profiles.item(0)

    cut_depth = wall_t - remaining_t
    if cut_depth <= 0:
        return  # remaining_t >= wall_t: niente da assottigliare

    extrudes = comp.features.extrudeFeatures
    extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extInput.participantBodies = [body]
    extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-mm(cut_depth)))
    extrudes.add(extInput)


# ============================================================
#  COSTRUZIONE DI UN DISPOSITIVO (Base + Lid)
# ============================================================

def build_device(root, name, L, W, H):
    comp = new_component(root, name)

    base = create_box_body(comp, "Base", L, W, H, CORNER_R)
    shell_open_top(comp, base, WALL_T)
    boss_h = H - BOSS_H_MARGIN
    hole_centers = add_corner_bosses(comp, base, L, W, WALL_T, BOSS_INSET,
                                      BOSS_D, SCREW_HOLE_D, boss_h)

    lid, lid_plane = create_lid(comp, "Lid", L, W, LID_T, CORNER_R, H,
                                 hole_centers, SCREW_HOLE_D)

    return comp, base, lid, lid_plane


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Apri o crea un documento Fusion 360 (Design) prima di lanciare lo script.')
            return
        root = design.rootComponent

        # ---- COLLARE (baccello IP65) ----
        comp_c, base_c, lid_c, lid_plane_c = build_device(root, "Collare", COLLARE_L, COLLARE_W, COLLARE_H)
        add_gasket_channel(comp_c, base_c, COLLARE_L, COLLARE_W, WALL_T, COLLARE_H,
                            GASKET_GROOVE_W, GASKET_GROOVE_DEPTH)
        add_mount_ears(comp_c, base_c, COLLARE_L, EAR_W, EAR_T, EAR_Z, EAR_H,
                       STRAP_SLOT_W, STRAP_SLOT_H)
        cut_lid_holes(comp_c, lid_c, lid_plane_c, LID_T,
                      [(COLLARE_ANT_CX, COLLARE_ANT_CY, ANTENNA_HOLE_D)])
        cut_side_slot(comp_c, base_c, "back", COLLARE_L, COLLARE_W, COLLARE_H,
                      USB_SLOT_W, USB_SLOT_H, USB_SLOT_Z, WALL_T + 1)
        thin_wall_spot(comp_c, base_c, "right", COLLARE_L, COLLARE_W,
                       REED_REMAINING_T, REED_SPOT_D, REED_SPOT_Z, WALL_T)

        # ---- PALMARE (stile radio, alto e stretto) ----
        comp_p, base_p, lid_p, lid_plane_p = build_device(root, "Palmare", PALMARE_L, PALMARE_W, PALMARE_H)
        cut_side_slot(comp_p, base_p, "front", PALMARE_L, PALMARE_W, PALMARE_H,
                      PALMARE_DISPLAY_W, PALMARE_DISPLAY_H, PALMARE_DISPLAY_Z, WALL_T + 1)
        cut_lid_holes(comp_p, lid_p, lid_plane_p, LID_T, [
            (PALMARE_ENC_CX, PALMARE_ENC_CY, PALMARE_ENC_HOLE_D),
            (PALMARE_ANT_LORA_CX, PALMARE_ANT_LORA_CY, ANTENNA_HOLE_D),
            (PALMARE_ANT_GPS_CX, PALMARE_ANT_GPS_CY, ANTENNA_HOLE_D),
        ])
        cut_side_slot(comp_p, base_p, "back", PALMARE_L, PALMARE_W, PALMARE_H,
                      USB_SLOT_W, USB_SLOT_H, USB_SLOT_Z, WALL_T + 1)
        # Il Palmare attuale si accende/spegne con l'encoder (click/pressione),
        # NON ha un reed switch/magnete - se la tua versione fisica ne ha uno,
        # decommenta la riga sotto e verifica il lato/altezza:
        # thin_wall_spot(comp_p, base_p, "right", PALMARE_L, PALMARE_W,
        #                REED_REMAINING_T, REED_SPOT_D, REED_SPOT_Z, WALL_T)

        ui.messageBox('Fatto: generati i componenti "Collare" e "Palmare".\n'
                       'Controlla le quote (sono placeholder, da verificare col calibro),\n'
                       'la direzione dei tagli laterali/canalina/alette, e ricorda che i\n'
                       'fori antenna da soli NON sono impermeabili: serve un pressacavo\n'
                       'o connettore SMA con O-ring inserito li\'.')
    except RuntimeError as e:
        if ui:
            if 'un solo componente' in str(e):
                ui.messageBox(
                    'Il documento attivo e\' un "Part" (documento a componente singolo): '
                    'questo tipo di documento puo\' contenere un solo componente e non va bene '
                    'per questo script, che ne crea due ("Collare" e "Palmare").\n\n'
                    'Soluzione: chiudi questo documento e crea un nuovo documento con '
                    'File > New Design (NON File > New Part), poi rilancia lo script.')
            else:
                ui.messageBox('Errore:\n{}'.format(traceback.format_exc()))
    except:
        if ui:
            ui.messageBox('Errore:\n{}'.format(traceback.format_exc()))
