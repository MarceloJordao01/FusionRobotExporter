#Author- FusionRobotExporter
#Description- Export Fusion 360 designs to URDF/SDF for robotics simulation

import adsk.core
import adsk.fusion
import traceback

# Global handlers to keep them referenced
handlers = []


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Create command definition
        cmd_def = ui.commandDefinitions.itemById('FusionRobotExporter')
        if cmd_def:
            cmd_def.deleteMe()

        cmd_def = ui.commandDefinitions.addButtonDefinition(
            'FusionRobotExporter',
            'Fusion Robot Exporter',
            'Export design to URDF or SDF format for robotics simulation'
        )

        # Connect to command created event
        on_command_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_command_created)
        handlers.append(on_command_created)

        # Execute the command
        cmd_def.execute()

        # Keep the script running while dialog is open
        adsk.autoTerminate(False)

    except:
        if ui:
            ui.messageBox(f'Failed:\n{traceback.format_exc()}')


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: adsk.core.CommandCreatedEventArgs):
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            # --- Export Format ---
            format_group = inputs.addGroupCommandInput('format_group', 'Export Format')
            format_group_children = format_group.children

            format_dropdown = format_group_children.addDropDownCommandInput(
                'export_format',
                'Format',
                adsk.core.DropDownStyles.TextListDropDownStyle
            )
            format_dropdown.listItems.add('URDF', True)
            format_dropdown.listItems.add('SDF', False)

            # --- ROS Version ---
            ros_group = inputs.addGroupCommandInput('ros_group', 'ROS Configuration')
            ros_group_children = ros_group.children

            ros_dropdown = ros_group_children.addDropDownCommandInput(
                'ros_version',
                'ROS Version',
                adsk.core.DropDownStyles.TextListDropDownStyle
            )
            ros_dropdown.listItems.add('ROS1', False)
            ros_dropdown.listItems.add('ROS2', True)

            # --- Link Configuration ---
            # Kept in its own (always visible) group because the base link
            # selection is also needed for SDF when using rigid-group mode.
            app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(app.activeProduct)

            link_group = inputs.addGroupCommandInput('link_group', 'Link Configuration')
            link_group_children = link_group.children

            # When checked, each Rigid Group becomes a single merged link and the
            # Base Link dropdown lists the rigid groups instead of occurrences.
            link_group_children.addBoolValueInput(
                'link_mode_rigid', 'Define links by Rigid Group', True, '', False)

            base_link_dropdown = link_group_children.addDropDownCommandInput(
                'base_link',
                'Base Link',
                adsk.core.DropDownStyles.TextListDropDownStyle
            )

            if design:
                root = design.rootComponent

                # Default (component mode): populate with occurrences.
                first = True
                for occ in root.allOccurrences:
                    base_link_dropdown.listItems.add(occ.name, first)
                    first = False

            # --- Export Options ---
            options_group = inputs.addGroupCommandInput('options_group', 'Export Options')
            options_group_children = options_group.children

            options_group_children.addBoolValueInput('export_meshes', 'Export Meshes', True, '', True)
            options_group_children.addBoolValueInput('export_collision', 'Generate Collision', True, '', True)
            options_group_children.addBoolValueInput('export_inertia', 'Calculate Inertia', True, '', True)
            options_group_children.addBoolValueInput('export_launch', 'Generate Launch Files', True, '', True)

            # --- Mesh Options ---
            mesh_group = inputs.addGroupCommandInput('mesh_group', 'Mesh Options')
            mesh_group_children = mesh_group.children

            mesh_format = mesh_group_children.addDropDownCommandInput(
                'mesh_format',
                'Mesh Format',
                adsk.core.DropDownStyles.TextListDropDownStyle
            )
            mesh_format.listItems.add('STL', True)
            mesh_format.listItems.add('OBJ', False)

            collision_type = mesh_group_children.addDropDownCommandInput(
                'collision_type',
                'Collision Geometry',
                adsk.core.DropDownStyles.TextListDropDownStyle
            )
            collision_type.listItems.add('Mesh', True)
            collision_type.listItems.add('Bounding Box', False)
            collision_type.listItems.add('Simplified', False)

            # --- Event handlers ---
            on_execute = CommandExecuteHandler()
            cmd.execute.add(on_execute)
            handlers.append(on_execute)

            on_destroy = CommandDestroyHandler()
            cmd.destroy.add(on_destroy)
            handlers.append(on_destroy)

            on_input_changed = CommandInputChangedHandler()
            cmd.inputChanged.add(on_input_changed)
            handlers.append(on_input_changed)

        except:
            app = adsk.core.Application.get()
            ui = app.userInterface
            ui.messageBox(f'Command created failed:\n{traceback.format_exc()}')


class CommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: adsk.core.InputChangedEventArgs):
        try:
            changed_input = args.input
            inputs = args.inputs

            # Show/hide ROS version based on format
            if changed_input.id == 'export_format':
                export_format = inputs.itemById('export_format').selectedItem.name
                ros_group = inputs.itemById('ros_group')

                # SDF doesn't need ROS version selection (it's ROS2/Gazebo native)
                if export_format == 'SDF':
                    ros_group.isVisible = False
                else:
                    ros_group.isVisible = True

                # Update mesh format based on export format
                mesh_format = inputs.itemById('mesh_format')
                if export_format == 'SDF':
                    mesh_format.listItems.item(1).isSelected = True  # OBJ for SDF
                else:
                    mesh_format.listItems.item(0).isSelected = True  # STL for URDF

            # Repopulate the Base Link dropdown when the link mode toggles
            if changed_input.id == 'link_mode_rigid':
                app = adsk.core.Application.get()
                design = adsk.fusion.Design.cast(app.activeProduct)
                base_link_dropdown = inputs.itemById('base_link')
                base_link_dropdown.listItems.clear()

                if design:
                    root = design.rootComponent
                    if changed_input.value:
                        # Rigid-group mode: list the rigid groups.
                        from .core import rigid_groups as core_rigid
                        first = True
                        for name in core_rigid.list_rigid_group_names(root):
                            base_link_dropdown.listItems.add(name, first)
                            first = False
                    else:
                        # Component mode: list the occurrences.
                        first = True
                        for occ in root.allOccurrences:
                            base_link_dropdown.listItems.add(occ.name, first)
                            first = False

        except:
            pass


class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: adsk.core.CommandEventArgs):
        try:
            app = adsk.core.Application.get()
            ui = app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)

            if not design:
                ui.messageBox('No active design. Please open a design first.')
                return

            # Get input values
            inputs = args.command.commandInputs

            export_format = inputs.itemById('export_format').selectedItem.name
            ros_version = inputs.itemById('ros_version').selectedItem.name
            base_link_input = inputs.itemById('base_link')
            base_link = base_link_input.selectedItem.name if base_link_input.selectedItem else None
            export_meshes = inputs.itemById('export_meshes').value
            export_collision = inputs.itemById('export_collision').value
            export_inertia = inputs.itemById('export_inertia').value
            export_launch = inputs.itemById('export_launch').value
            mesh_format = inputs.itemById('mesh_format').selectedItem.name
            collision_type = inputs.itemById('collision_type').selectedItem.name
            link_mode = 'rigid_groups' if inputs.itemById('link_mode_rigid').value else 'components'

            # Ask for save location
            folder_dialog = ui.createFolderDialog()
            folder_dialog.title = f'Select folder to save {export_format}'
            result = folder_dialog.showDialog()

            if result != adsk.core.DialogResults.DialogOK:
                return

            save_path = folder_dialog.folder

            # Build options dict
            options = {
                'base_link': base_link,
                'export_meshes': export_meshes,
                'export_collision': export_collision,
                'export_inertia': export_inertia,
                'export_launch': export_launch,
                'mesh_format': mesh_format,
                'collision_type': collision_type,
                'link_mode': link_mode,
            }

            # Call appropriate exporter
            success = False
            message = ''

            if export_format == 'URDF':
                if ros_version == 'ROS1':
                    from .exporters.urdf_ros1 import exporter
                    success, message = exporter.export(design, save_path, options)
                else:  # ROS2
                    from .exporters.urdf_ros2 import exporter
                    success, message = exporter.export(design, save_path, options)
            else:  # SDF
                from .exporters.sdf import exporter
                success, message = exporter.export(design, save_path, options)

            # Show result
            if success:
                ui.messageBox(message, 'FusionRobotExporter - Success')
            else:
                ui.messageBox(f'Export failed:\n{message}', 'FusionRobotExporter - Error')

        except:
            app = adsk.core.Application.get()
            ui = app.userInterface
            ui.messageBox(f'Execute failed:\n{traceback.format_exc()}')


class CommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args: adsk.core.CommandEventArgs):
        adsk.terminate()
