# frozen_string_literal: true

require "syskit/network_generation"

module Transformer
    # Module used to add some functionality to Syskit::NetworkGeneration::Engine
    module SystemNetworkGeneratorExtension
        # During network validation, checks that all required frames have been
        # configured
        def validate_generated_network(error_handler: @error_handler)
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
                rescue BaseException => e
                    @error_handler.register_resolution_failures_from_exception(task, e)
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
                    invalid_chain = InvalidChain.new(
                        placeholder_task.transformer,
                        task, task_from, from, task_to, to, e
                    )
                    message = "cannot find a transformation chain to produce " \
                              "#{from} => #{to} for #{task} (task-local frames: " \
                              "#{task_from} => #{task_to}): #{e.message}"
                    invalid_chain = invalid_chain.exception(message)
                    invalid_chain.set_backtrace(e.backtrace)
                    @error_handler
                        .register_resolution_failures_from_exception(task, invalid_chain)
                end
        end

        def validate_abstract_network(error_handler: @error_handler)
            super

            if Syskit.conf.transformer_enabled?
                # We must now validate. The frame propagation algorithm does
                # some validation, but also tries to do as little work as
                # possible and therefore will miss some errors
                transformer_tasks = plan.find_local_tasks(Syskit::TaskContext).
                    find_all { |task| task.model.transformer }
                transformer_tasks.each do |task|
                    SystemNetworkGeneratorExtension
                        .validate_frame_selection_consistency_through_inputs(
                            task, error_handler
                        )
                end
            end
        end

        def self.validate_mismatched_annotated_input(task, error_handler)
            task.each_annotated_port do |task_port, task_frame|
                next if !task_port.input? || !task_frame

                task_port.each_frame_of_connected_ports
                         .find_all { |other_port, other_frame| other_frame != task_frame }
                         .each do |port, frame|
                             conflict = FrameSelectionConflict.new(
                                task,
                                task.model.find_frame_of_port(task_port),
                                task_frame,
                                frame
                             )
                             error_handler.register_resolution_failures_from_exception(
                                task, conflict
                             )
                         end
            end
        end

        def self.validate_mismatched_from_frame_in_transform_input(
            inputs_from, task, error_handler
        )
            inputs_from.each do |task_port, tf|
                mismatch =
                    task_port
                    .each_transform_of_connected_ports
                    .find_all { |_, other_tf| other_tf.from && other_tf.from != tf.from }
                mismatch.each do |_, other_tf|
                    task_local_name = task.model.find_transform_of_port(task_port).from
                    conflict = FrameSelectionConflict.new(task, task_local_name,
                                                          tf.from, other_tf.from)
                    error_handler
                        .register_resolution_failures_from_exception(task, conflict)
                end
            end
        end

        def self.validate_mismatched_to_frame_in_transform_input(
            inputs_to, task, error_handler
        )
            inputs_to.each do |task_port, tf|
                mismatch =
                    task_port
                    .each_transform_of_connected_ports
                    .find_all { |_, other_tf| other_tf.to && other_tf.to != tf.to }
                mismatch.each do |_, other_tf|
                    task_local_name = task.model.find_transform_of_port(task_port).to
                    conflict = FrameSelectionConflict.new(task, task_local_name,
                                                          tf.to, other_tf.to)
                    error_handler
                        .register_resolution_failures_from_exception(task, conflict)
                end
            end
        end

        def self.validate_mismatched_transform_input(task, error_handler)
            inputs = task.each_transform_port.find_all { |task_port, _| task_port.input? }

            inputs_with_from = inputs.find_all { |_, transform| transform.from }
            inputs_with_to = inputs.find_all { |_, transform| transform.to }
            validate_mismatched_from_frame_in_transform_input(inputs_with_from, task,
                                                              error_handler)
            validate_mismatched_to_frame_in_transform_input(inputs_with_to, task,
                                                            error_handler)
        end

        def self.validate_frame_selection_consistency_through_inputs(task, error_handler)
            validate_mismatched_annotated_input(task, error_handler)
            validate_mismatched_transform_input(task, error_handler)
        end
    end
end

