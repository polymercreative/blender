/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 *
 * Fix Poles Modifier - Fixes 3-edge poles in meshes by collapsing them.
 * Based on the fix_poles algorithm from sculpt mode remeshing.
 */

#include "BLI_utildefines.h"

#include "BLT_translation.hh"

#include "DNA_defaults.h"
#include "DNA_screen_types.h"

#include "BKE_mesh.hh"
#include "BKE_mesh_remesh_voxel.hh"
#include "BKE_modifier.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_prototypes.hh"
#include "RNA_types.hh"

#include "MOD_ui_common.hh"

static void init_data(ModifierData *md)
{
  FixPolesModifierData *fpmd = (FixPolesModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(fpmd, modifier));

  MEMCPY_STRUCT_AFTER(fpmd, DNA_struct_default_get(FixPolesModifierData), modifier);
}

static Mesh *modify_mesh(ModifierData * /*md*/, const ModifierEvalContext * /*ctx*/, Mesh *mesh)
{
  Mesh *result = BKE_mesh_remesh_voxel_fix_poles(mesh);
  if (result == nullptr) {
    return mesh;
  }

  BKE_mesh_copy_parameters_for_eval(result, mesh);

  return result;
}

static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  layout->use_property_split_set(true);

  /* No properties for this simple modifier */
  layout->label(N_("Fixes 3-edge poles by collapsing them"), ICON_NONE);

  modifier_error_message_draw(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_FixPoles, panel_draw);
}

ModifierTypeInfo modifierType_FixPoles = {
    /*idname*/ "FixPoles",
    /*name*/ N_("Fix Poles"),
    /*struct_name*/ "FixPolesModifierData",
    /*struct_size*/ sizeof(FixPolesModifierData),
    /*srna*/ &RNA_FixPolesModifier,
    /*type*/ ModifierTypeType::Constructive,
    /*flags*/ eModifierTypeFlag_AcceptsMesh | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_AcceptsCVs,
    /*icon*/ ICON_MOD_REMESH,

    /*copy_data*/ BKE_modifier_copydata_generic,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ modify_mesh,
    /*modify_geometry_set*/ nullptr,

    /*init_data*/ init_data,
    /*required_data_mask*/ nullptr,
    /*free_data*/ nullptr,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ nullptr,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ nullptr,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ panel_register,
    /*blend_write*/ nullptr,
    /*blend_read*/ nullptr,
    /*foreach_cache*/ nullptr,
    /*foreach_working_space_color*/ nullptr,
};
