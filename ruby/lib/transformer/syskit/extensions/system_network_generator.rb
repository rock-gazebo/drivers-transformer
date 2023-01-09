module Transformer
    # Module used to add some functionality to Syskit::NetworkGeneration::Engine
    module SystemNetworkGeneratorExtension
        # During network validation, checks that all required frames have been
        # configured
        def validate_generated_network
            super

            return unless Syskit.conf.transformer_enabled?

            transformer_validate_all_frames_assigned
            transformer_validate_no_missing_transform
        end

        def transformer_validate_all_frames_assigned
            plan.find_local_tasks(Syskit::TaskContext).each do |task|
                next unless (tr = task.model.transformer)

                tr.each_needed_transformation do |transform|
                    transformer_validate_frame_assigned(task, transform.from)
                    transformer_validate_frame_assigned(task, transform.to)
                end
            end
        end

        def transformer_validate_frame_assigned(task, frame_name)
            return if task.selected_frames[frame_name]

            raise MissingFrame,
                  "could not find a frame assignment for #{frame_name} in #{task}"
        end

        def transformer_validate_no_missing_transform
            plan.find_local_tasks(Transformer::SyskitPlugin::MissingTransform)
                .each do |placeholder_task|
                    task = placeholder_task.parent_task
                    task_from = placeholder_task.task_from
                    from = placeholder_task.from
                    task_to = placeholder_task.task_to
                    to = placeholder_task.to
                    e = placeholder_task.original_exception
                    raise InvalidChain.new(
                        placeholder_task.transformer,
                        task, task_from, from, task_to, to, e
                    ),
                          "cannot find a transformation chain to produce "\
                          "#{from} => #{to} for #{task} (task-local frames: "\
                          "#{task_from} => #{task_to}): #{e.message}", e.backtrace
                end
        end

        def validate_abstract_network
            super

            if Syskit.conf.transformer_enabled?
                # We must now validate. The frame propagation algorithm does
                # some validation, but also tries to do as little work as
                # possible and therefore will miss some errors
                transformer_tasks = plan.find_local_tasks(Syskit::TaskContext).
                    find_all { |task| task.model.transformer }
                transformer_tasks.each do |task|
                    SystemNetworkGeneratorExtension.validate_frame_selection_consistency_through_inputs(task)
                end
            end
        end

        def self.validate_frame_selection_consistency_through_inputs(task)
            task.each_annotated_port do |task_port, task_frame|
                next if !task_port.input? || !task_frame
                task_port.each_frame_of_connected_ports do |other_port, other_frame|
                    if other_frame != task_frame
                        raise FrameSelectionConflict.new(
                            task,
                            task.model.find_frame_of_port(task_port),
                            task_frame,
                            other_frame)
                    end
                end
            end
            task.each_transform_port do |task_port, task_transform|
                next if !task_port.input?
                task_port.each_transform_of_connected_ports do |other_port, other_transform|
                    if other_transform.from && task_transform.from && other_transform.from != task_transform.from
                        task_local_name = task.model.find_transform_of_port(task_port).from
                        raise FrameSelectionConflict.new(task, task_local_name,
                                                         task_transform.from, other_transform.from)
                    elsif other_transform.to && task_transform.to && other_transform.to != task_transform.to
                        task_local_name = task.model.find_transform_of_port(task_port).to
                        raise FrameSelectionConflict.new(task, task_local_name,
                                                         task_transform.to, other_transform.to)
                    end
                end
            end
        end
    end
end

