# -*- coding: utf-8 -*-
"""
Main exporter for SDF format
Based on FusionSDF by andreasBihlmaier
"""

import adsk
import adsk.core
import adsk.fusion
from pathlib import Path

from .log import set_log_console, log
from .sdf import SDF
from ...core.progress import ProgressReporter, count_link_occurrences
from ...core.rigid_groups import count_rigid_groups_bodies


def export(design, save_dir, options=None):
    """
    Export Fusion 360 design to SDF format

    Parameters
    ----------
    design: adsk.fusion.Design - The active design
    save_dir: str - Directory to save the exported files
    options: dict - Export options (optional)

    Returns
    ----------
    tuple: (success: bool, message: str)
    """
    options = options or {}
    progress = None

    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        progress = ProgressReporter(ui)

        # Setup logging to Fusion console. We do NOT force the Text Commands
        # palette open: if the user already has it visible the diagnostics show
        # up there, otherwise we stay out of the way. (Forcing it open on every
        # run was noisy, and repainting it per body slowed large assemblies.)
        console = ui.palettes.itemById('TextCommands')
        set_log_console(console)

        log('\n\n--- FusionRobotExporter (SDF) ---\n')

        if not design:
            return False, 'No active Fusion design'

        sdf_dir_path = Path(save_dir)
        if not sdf_dir_path.exists() or not sdf_dir_path.is_dir():
            return False, f'"{sdf_dir_path}" does not exist or is not a directory'

        # Check for existing files
        if sdf_dir_path.joinpath('model.sdf').exists() or sdf_dir_path.joinpath('meshes').exists():
            # In the full implementation, would ask user for confirmation
            pass

        # Use mesh cache if available
        meshes_cache_path = sdf_dir_path / 'meshes_cache'
        if meshes_cache_path.exists():
            log(f'Using meshes_cache directory "{meshes_cache_path}"\n')
        else:
            meshes_cache_path = None

        # Generate SDF
        link_mode = options.get('link_mode', 'components')

        root = design.rootComponent
        if link_mode == 'rigid_groups':
            # The body merge is the heavy phase here, so progress stays body-based.
            total = count_rigid_groups_bodies(root)
            unit, message = 'bodies', 'Merging rigid-group bodies...'
        else:
            # One mesh per link (whole occurrence), like URDF -> count links.
            total = count_link_occurrences(root)
            unit, message = 'links', 'Exporting links...'
        progress.start(total, 'Exporting SDF', message, unit=unit)

        sdf = SDF(design, meshes_cache_path, link_mode, options.get('base_link'), progress=progress)

        if progress.is_cancelled():
            return False, 'Export cancelled by user.'

        sdf.print()
        sdf.save(sdf_dir_path)

        return True, f'Successfully exported SDF to {save_dir}'

    except Exception as e:
        import traceback
        return False, f'Export failed: {str(e)}\n{traceback.format_exc()}'
    finally:
        if progress is not None:
            progress.finish()
