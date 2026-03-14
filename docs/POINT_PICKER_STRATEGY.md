# Point Picker Enhancement: Strategy, Roadmap & Architecture

## 1. Executive summary

This document outlines how to extend this CloudCompare fork so that:

1. **Code list workflow**: You can load a list of point codes (from your graph software), then when picking points on the cloud you **choose a code from that list** instead of typing. The UI helps you work through the list so you can see what’s done and avoid missing or misspelling codes.

2. **Project-wide label export**: You can **export all point labels** from the current project in one go (one file), instead of exporting per cloud/segment.

3. **Unified picking across all clouds (Phase 1.5)**: You can run Point List Picking without selecting a single cloud first. You click anywhere on the scene; the pick is resolved to the cloud under the cursor (CloudCompare’s existing pick gives you the entity). Each new label is stored in a **single project-level list** and shown in one main table. Export all then exports that full list in one go.

The design reuses the existing Point List Picking and label infrastructure where possible, and adds a small number of new entry points and data flows.

---

## 2. Current behaviour (baseline)

### 2.1 Point list picking

- **Location**: `qCC/ccPointListPickingDlg.{h,cpp}`, `qCC/ccPointPickingGenericInterface.h`, UI `qCC/ui_templates/pointListPickingDlg.ui`.
- **Trigger**: Tools → Point list picking (or equivalent). Requires **exactly one** selected entity (one point cloud or one mesh).
- **Flow**: User clicks on the cloud; each click creates a **cc2DLabel** with one picked point. Labels are stored in a child container of that entity named `"Picked points list"` (see `s_pickedPointContainerName`). Default label name is `"Point #" + (startIndex + i)`.
- **Naming**: In `updateList()`, any label that already has a custom name (not the default `"Point #…"`) is left unchanged; new labels get the default. So custom names are possible (e.g. via properties), but there is no in-dialog way to pick from a predefined list.
- **Export**: Export (ASCII xyz / ixyz / gxyz / **lxyz** = label name, x, y, z) only exports labels from the **current** entity’s “Picked points list”. There is no “export all labels in the project”.

### 2.2 Labels in the scene

- **Type**: `cc2DLabel` (libs/qCC_db, `cc2DLabel.{h,cpp}`). Single-point labels have `size() == 1` and store a `PickedPoint` (cloud/mesh + index + optional UV for meshes).
- **Placement**: Labels are children of a hierarchy object (typically under a specific cloud’s “Picked points list”). The DB root is available via `MainWindow::TheInstance()->dbRootObject()` (returns `ccHObject*`). All 2D labels in the project can be collected with:
  - `root->filterChildren(container, true, CC_TYPES::LABEL_2D)` (recursive).
- **Coordinates**: `PickedPoint::getPointPosition()` returns local coordinates. Global coordinates (e.g. lat/long/depth) require the parent cloud’s shift/scale (e.g. `ccShiftedObject` / `toGlobal3d()`), which is already used in the existing export and in the “show global coordinates” option.

### 2.3 Entry points

- Point list picking is started from `MainWindow::activatePointListPickingMode()` in `qCC/mainwindow.cpp` (around 7042). It enforces “exactly one entity” and passes that entity to `ccPointListPickingDlg::linkWithEntity()`.
- Export is only from within the dialog, and only for `m_orderedLabelsContainer` of the linked entity.

---

## 3. Goals and requirements

| # | Requirement | Notes |
|---|-------------|--------|
| R1 | Load a **list of point codes** (from graph software). | Format TBD: e.g. one code per line, or CSV (code, optional lat, long, depth). |
| R2 | When picking a point on the cloud, **assign a label by selecting a code from that list** (no typing). | Reduces typos and keeps labels aligned with the graph. |
| R3 | **Checklist / progress** over the code list so the user can see which codes are already assigned and ensure none are missed or misspelled. | e.g. list view with “used”/“unused” or checkmarks. |
| R4 | **Export all point labels** in the project **in one go** to a single file. | Not per cloud/segment; one export for the whole scene. |
| R5 | Export format compatible with downstream script (e.g. label + lat, long, depth in metres). | Same or similar to current lxyz-style export, with global coordinates where needed. |
| R6 | **Single picking list across all clouds**: Pick on any visible cloud; label is attached to the correct cloud in the background; one main list shows all picks; export all in one file. | No need to select “one entity” before starting; no per-cloud export. |

Non-goals for this roadmap: changing how graph software works, or implementing distance/direction/slope (those stay in the user’s existing script).

---

## 4. High-level architecture

### 4.1 Code list and “pick from list” workflow

- **Data**: A **code list** (ordered set of strings) loaded from file and kept in memory (e.g. in the point list picking dialog or a small helper).
- **UI**: In the Point List Picking dialog (or an extended version):
  - **Load list**: Button/menu to load a file → parse into list of codes.
  - **List panel**: Show the list of codes with a “used” state (e.g. which code was assigned to which pick, or simply “used”/“unused”). Optionally allow reordering or filtering.
  - **On pick**: When the user clicks a point, instead of (or in addition to) creating a label with a default name:
    - Open a small chooser (e.g. combo box or list widget) pre-filled with the loaded codes.
    - User selects one code → create (or update) the label with that name; mark that code as used.
  - **Persistence**: Code list can be session-only; optionally remember last path for “load list”.
- **Label naming**: Use the selected code as the label’s name (`cc2DLabel` inherits `ccHObject::setName()`). Ensure the dialog’s `updateList()` (or equivalent) does **not** overwrite user-chosen names with `"Point #…"` when a code was selected. The existing logic already preserves non-default names for “old” labels; the same idea applies to “code-assigned” names.

### 4.2 Project-wide label export

- **Entry point**: New menu action, e.g. **File → Export → Export all point labels** (or under a dedicated “Labels” submenu). No need to have the Point List Picking dialog open; works on the current DB state.
- **Logic**:
  1. Get root: `MainWindow::TheInstance()->dbRootObject()`.
  2. Collect all 2D labels: `root->filterChildren(labels, true, CC_TYPES::LABEL_2D)`.
  3. Filter to **point labels** only: keep only labels with `size() == 1` (single-point labels).
  4. For each label: get `getPickedPoint(0)`, then `getPointPosition()` and the owning entity’s global shift/scale to compute global coordinates (lat, long, depth in metres if that’s the convention).
  5. Write one file (e.g. CSV or tab-separated): at least `label_name, x, y, z` (or `label_name, lat, long, depth`); format chosen to match the user’s downstream script.
- **File format**: Prefer configurable (e.g. dropdown: “label,x,y,z” vs “label,lat,long,depth”) and consistent with existing ASCII export (e.g. `AsciiFilter`-style or same columns as lxyz). Reuse existing “apply global shift/scale?” behaviour where applicable.

### 4.3 Unified picking across all clouds (Phase 1.5)

- **Goal**: One Point List Picking session where the user does **not** have to select a single cloud first. They click a point; the system uses the **entity under the cursor** (CloudCompare’s pick already returns the hit entity). A label is created and stored in a **single project-level container**; the dialog shows **one main list** of all picks (from any cloud). “Export all point labels” then exports that full list in one file.
- **Behaviour**:
  - **Pick anywhere**: Picking is enabled on **any visible, pickable** point cloud or mesh in the scene. No “exactly one entity selected” requirement to enter the tool.
  - **Best guess = hit entity**: The picking hub / `PickedItem` already identifies which entity was clicked; no extra heuristics. The label’s `PickedPoint` references that entity and index, so coordinates (and later global coords for export) stay correct per source cloud.
  - **Single container**: Create one root-level container (e.g. `"Project picked points"`) under `dbRootObject()`. All new labels from this mode are added there. The dialog’s “main list” reads from this container only (or aggregates it with existing per-entity lists for backward compatibility—see below).
  - **One list in the UI**: The Point List Picking table shows all labels from the project-level container (and optionally a “Source” column: which cloud the point belongs to). Export all continues to walk the whole DB for labels, so the single file includes everything.
- **Backward compatibility**: Existing “Picked points list” children under individual clouds can remain. “Export all point labels” already collects every 2D point label in the tree, so old per-cloud lists and the new project-level list are all exported together. Optionally, the dialog can offer a “mode”: “Single cloud (current)” vs “All clouds (project list)” and create the project-level container only when in “All clouds” mode.
- **Implementation touchpoints**:
  - **MainWindow::activatePointListPickingMode()**: When in “unified” mode, do not require one selection; allow zero or N selected; pass “all clouds” or root to the dialog (e.g. `linkWithEntity(nullptr)` and a flag, or `linkWithRoot()`).
  - **ccPointListPickingDlg**: Support a second mode where `m_orderedLabelsContainer` is the **project-level** container (create once under root, reuse). In `processPickedPoint()`, accept any cloud/mesh (not only `m_associatedEntity`); create the label from `picked.entity` and add it to the project-level container.
  - **Picking**: Ensure the picking hub can deliver picks from any visible entity when no single entity is “linked”; this may already work if the dialog simply does not filter by entity in the listener.

---

## 5. Component design

### 5.1 Code list

- **Class**: Either a simple `QStringList` (or `std::vector<QString>`) in the dialog, or a small class e.g. `PointCodeList` that holds:
  - `QStringList codes`;
  - optional `QSet<QString> usedCodes` (or a map code → cc2DLabel* for “which label has this code”).
- **File format**: Keep it simple in v1, e.g.:
  - One code per line (trimmed, skip empty).
  - Or CSV: first column = code, optional columns = lat, long, depth (for future validation/display only).
- **Storage**: Member of `ccPointListPickingDlg` (or of a derived class if we split “basic” vs “code list” mode).

### 5.2 Unified picking (Phase 1.5): project-level container and dialog mode

- **Project-level container**: A single `ccHObject` (e.g. name `"Project picked points"`) stored under `dbRootObject()`. Created when the user first uses “Point list picking (all clouds)” or when the dialog is in unified mode; reused for the session. All new labels in that mode are `addChild()` here; display is set so labels appear in the 3D view.
- **Dialog mode**: Dialog has two logical modes: (1) **Single entity** (current): `linkWithEntity(entity)`; container = that entity’s “Picked points list”. (2) **Unified**: no single entity; container = root-level “Project picked points”. The main table always shows the labels from the *current* container; in unified mode that is the project list.
- **processPickedPoint()**: In unified mode, accept any cloud/mesh in `picked.entity`; create label from `picked.entity` and `picked.itemIndex` (and mesh UV if needed); append to project-level container; refresh table. Optionally add a “Source” column (e.g. `picked.entity->getName()`) so the user sees which cloud each row came from.
- **Picking hub**: Confirm that when no single entity is linked, the hub still delivers picks from whatever is under the cursor (or explicitly enable picking on all visible clouds when in unified mode).

### 5.3 UI changes (Point List Picking dialog) – code list (Phase 2+)

- **New widgets** (in `pointListPickingDlg.ui` or a separate small dialog):
  - “Load code list” button → file dialog → load and parse.
  - Optional: “Clear code list”.
  - A list/table showing codes and “Used”/“Unused” (and optionally which label name they have). This can be a second panel or a collapsible section.
  - When a point is picked: if code list is loaded, show a **code selector** (e.g. `QComboBox` with the list, or a `QListWidget` with search); on “OK” create the label with the selected code as name and mark that code as used. If no code list is loaded, keep current behaviour (default “Point #N”).
- **Table**: The existing table (Index, X, Y, Z) can stay; in unified mode add optional “Source” column; optionally add a “Label name” column that shows the code (already reflected by `labels[i]->getName()` in export). The “start index” and “marker size” behaviour can remain.

### 5.4 Export all labels

- **New function**: e.g. `MainWindow::exportAllPointLabels()`.
  - Collect all `cc2DLabel` with `size() == 1` from `dbRootObject()`.
  - Resolve global coordinates using each label’s `PickedPoint::entity()` and its shift/scale (same as in `ccPointListPickingDlg::exportToASCII` for lxyz).
  - Ask user for file path and optional format (label,x,y,z vs label,lat,long,depth if different).
  - Write one file.
- **Menu**: Register in `mainwindow.cpp` and in the UI (e.g. `mainWindow.ui` or a menu bar action). Enable when the DB is not empty (or always); disable when there are no point labels if desired.

### 5.5 Data flow (unified picking + code list + export)

```
[Phase 1.5] User starts "Point list picking (all clouds)" → no single-cloud selection
       ↓
User clicks on any visible cloud → pick resolves to that entity → label created and added to "Project picked points" → one main list updated
       ↓
[Phase 2+] Load code list → pick point → choose code from list → label name = code; checklist updated
       ↓
File → Export all point labels → collect all cc2DLabel (size==1) from root (incl. project container + any per-cloud lists) → one file (label, x, y, z or lat, long, depth)
       ↓
[Downstream script] → reads export, computes distance/direction/slope for pairs that have a connecting line
```

---

## 6. Roadmap (phased)

**Order**: Phase 1 (export all) → Phase 1.5 (unified picking across all clouds) → Phase 2 (code list load) → Phase 3 (pick from list + checklist) → Phase 4 (polish). Phase 1 delivers immediate value; Phase 1.5 delivers the single-list, pick-anywhere workflow; later phases add the code list and checklist.

### Phase 1 – Export all point labels (foundation)

- **1.1** Add **Export all point labels**:
  - Implement `MainWindow::exportAllPointLabels()` (or equivalent) that:
    - Uses `dbRootObject()->filterChildren(..., true, CC_TYPES::LABEL_2D)`.
    - Keeps only single-point labels.
    - Resolves global coordinates (shift/scale) and writes one ASCII file (e.g. label,x,y,z).
  - Add menu action and wire it in the main window.
- **1.2** Tests: Load a project with labels under two different clouds; run “Export all point labels”; confirm one file with all of them and correct coordinates.

**Deliverable**: One menu action to export every point label in the project to a single file.

### Phase 1.5 – Unified picking across all clouds

- **1.5.1** Introduce a **project-level label container** (e.g. name `"Project picked points"`) under the DB root. Create it on first use in “unified” mode; reuse for the session. All labels created in this mode are children of this container (not of individual clouds).
- **1.5.2** **MainWindow**: Add a way to start Point List Picking in “unified” mode (e.g. a second menu action “Point list picking (all clouds)” or a checkbox in the dialog). In this mode, do **not** require exactly one selected entity; allow starting with no selection or multiple. Optionally auto-enable picking on all visible point clouds/meshes.
- **1.5.3** **ccPointListPickingDlg**:
  - When in unified mode, set or create `m_orderedLabelsContainer` as the root-level container (e.g. from a new `linkWithRoot()` or `setUnifiedMode(true)` plus root pointer). Do not require `m_associatedEntity`.
  - In `processPickedPoint()`: accept any `picked.entity` that is a point cloud or mesh (not only `m_associatedEntity`). Create the `cc2DLabel` from the picked entity/index, add it to the project-level container, and refresh the single main list.
  - Ensure the main table lists all labels from the project container; optionally add a “Source” column (entity name) so the user can see which cloud each point came from.
- **1.5.4** **Picking**: Verify that the picking hub delivers clicks on any visible entity when no single entity is linked (e.g. by not filtering in the listener, or by registering for “all” entities). Adjust if the current implementation only forwards picks for a single entity.
- **1.5.5** “Export all point labels” (Phase 1) already iterates over all labels under the root, so the new project-level container is included automatically. No export changes required for Phase 1.5.

**Deliverable**: User can open Point List Picking without selecting one cloud; click on any cloud to add a point; see one combined list; export all labels (from this list and any existing per-cloud lists) in one file.

### Phase 2 – Code list load and display

- **2.1** Define code list file format (e.g. one code per line, UTF-8).
- **2.2** In Point List Picking dialog: add “Load code list” and a simple list widget showing loaded codes (no “used” state yet).
- **2.3** Persist “last used path” for the code list file (e.g. QSettings).

**Deliverable**: User can load a code list and see it in the dialog.

### Phase 3 – Pick from list and checklist

- **3.1** On point pick: if a code list is loaded, show a selector (combo or list) with the loaded codes; user selects one → create label with that name (`setName(selectedCode)`).
- **3.2** Ensure `updateList()` (and any auto-naming) does not overwrite these names (treat “code-assigned” names like existing custom names).
- **3.3** Track which codes have been assigned (e.g. set or map). Update the code list widget to show “Used”/“Unused” (e.g. checkmark or colour).
- **3.4** Optional: “Next suggested code” (e.g. first unused in list) to speed up sequential picking.

**Deliverable**: Full “load list → pick point → choose code → see progress” workflow.

### Phase 4 – Polish and format options

- **4.1** Export all: add format options (e.g. “Local x,y,z” vs “Global lat, long, depth”) and possibly delimiter (comma vs tab).
- **4.2** Code list: support CSV (first column = code) and “one per line”; optional columns for lat/long/depth for display only.
- **4.3** Small UX improvements: keyboard shortcut for “Export all point labels”, tooltips, and any needed translation hooks.

**Deliverable**: Robust, configurable export and flexible code list import.

---

## 7. File and class changes (summary)

| Area | File(s) | Change |
|------|--------|--------|
| Export all (Phase 1) | `qCC/mainwindow.{h,cpp}` | New method `exportAllPointLabels()`; new menu action. |
| Export all | `qCC/ui_templates/mainWindow.ui` (or menu bar) | Add “Export all point labels” action. |
| Unified picking (Phase 1.5) | `qCC/mainwindow.{h,cpp}` | Second entry for “Point list picking (all clouds)” or flag; do not require one selection when in unified mode; pass root or “all” to dialog. |
| Unified picking | `qCC/ccPointListPickingDlg.{h,cpp}` | Support unified mode: project-level container under root; `processPickedPoint()` accept any cloud/mesh and add to that container; optional “Source” column in table. |
| Code list (Phase 2+) | `qCC/ccPointListPickingDlg.{h,cpp}` | Members for code list + “used” set; “Load code list” and code selector on pick. |
| Code list | `qCC/ui_templates/pointListPickingDlg.ui` | Widgets: Load list button, code list widget, optional “used” column. |
| Shared | `libs/qCC_io/` or `qCC/` | Reuse existing ASCII/export helpers for global coords and file writing where possible. |

New classes are only needed if we want a separate “PointCodeList” or a dedicated “Export all labels” dialog; otherwise, logic can live in `MainWindow` and `ccPointListPickingDlg`.

---

## 8. Risks and mitigations

| Risk | Mitigation |
|------|------------|
| Global coordinates differ per cloud (shift/scale) | Reuse the same resolution as in `ccPointListPickingDlg::exportToASCII` (per-label entity and its shift/scale). |
| Many labels → slow or large export | Single pass over `filterChildren` is O(n); file I/O is sequential. For very large projects, consider progress bar. |
| Code list duplicates (same code twice) | Allow duplicates in the list but track “used” per occurrence or show a warning when loading. |
| User picks point but cancels code selector | Don’t create a label; or create with default name and let user rename. Define desired behaviour in Phase 3. |

---

## 9. Success criteria

- User can load a list of point codes from a file and see them in the Point List Picking dialog.
- User can pick a point and assign a label by selecting a code from that list (no typing).
- User can see which codes have been assigned (checklist) and export all point labels in the project to one file with correct global coordinates.
- Downstream script can consume the exported file (e.g. label, lat, long, depth) and compute distance/direction/slope for connected pairs as it does today.

---

## 10. References (in-repo)

- Point list picking: `qCC/ccPointListPickingDlg.cpp` (e.g. `linkWithEntity`, `processPickedPoint`, `exportToASCII`, `updateList`).
- Picking hub / listener: `qCC/ccPointPickingGenericInterface.h`, `ccPickingListener`.
- Labels: `libs/qCC_db/include/cc2DLabel.h` (`PickedPoint`, `getPointPosition`, `getName`/`setName`).
- Global label collection: `qCC/ccGraphicalSegmentationTool.cpp` (e.g. around 1544–1547) uses `dbRootObject()->filterChildren(..., true, CC_TYPES::LABEL_2D)`.
- Main window and DB: `qCC/mainwindow.{h,cpp}`, `qCC/db_tree/ccDBRoot.h` (`getRootEntity()`).
