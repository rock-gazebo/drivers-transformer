require 'transformer/syskit/test'

describe Transformer::SyskitPlugin do
    attr_reader :transform_producer_m, :data_producer_m, :data_consumer_m
    attr_reader :cmp_m
    before do
        Roby.app.import_types_from 'base'
        Roby.app.import_types_from 'transformer'
        Roby.app.using_task_library 'transformer'
        Syskit.conf.transformer_warn_about_unset_frames = false

        @transform_producer_m = Syskit::TaskContext.new_submodel name: 'TransformProducer' do
            property 'producer_object_frame', '/std/string'
            property 'producer_world_frame', '/std/string'
            output_port 'transform', '/base/samples/RigidBodyState'
            transformer do
                transform_output 'transform', 'producer_object' => 'producer_world'
                max_latency 0.1
            end
        end
        @data_producer_m = Syskit::TaskContext.new_submodel name: 'DataProducer' do
            property 'producer_object_frame', '/std/string'
            property 'producer_world_frame', '/std/string'
            output_port 'samples', '/double'
            output_port 'transform_samples', '/base/samples/RigidBodyState'
            transformer do
                associate_ports_to_frame "samples", "producer_object"
                transform_output 'transform_samples',
                    'producer_object' => 'producer_world'
                max_latency 0.1
            end
        end
        @data_consumer_m = Syskit::TaskContext.new_submodel name: 'DataConsumer' do
            property 'object_frame', '/std/string'
            property 'world_frame', '/std/string'
            input_port 'samples', '/double'
            input_port 'transform_samples', '/base/samples/RigidBodyState'
            transformer do
                transform 'object', 'world'
                associate_ports_to_frame "samples", "object"
                transform_input 'transform_samples',
                    'object' => 'world'
                max_latency 0.1
            end
        end
        @cmp_m = Syskit::Composition.new_submodel
    end

    it "assigns the required static transformations" do
        task_m = self.data_consumer_m.
            use_frames('object' => 'object_global', 'world' => 'world_global').
            transformer do
                static_transform Eigen::Vector3.new(1, 0, 0),
                    "object_global" => "world_global"
            end
        task = syskit_stub_deploy_and_configure(task_m)

        statics = task.properties.static_transformations
        assert_equal 1, statics.size
        assert_equal Eigen::Vector3.new(1, 0, 0), statics.to_a.first.position
        assert_equal Eigen::Quaternion.Identity, statics.to_a.first.orientation
    end

    it "assigns the frame name properties" do
        task_m = self.data_consumer_m.
            use_frames('object' => 'object_global', 'world' => 'world_global').
            transformer do
                static_transform Eigen::Vector3.new(1, 0, 0),
                    "object_global" => "world_global"
            end
        task = syskit_stub_deploy_and_configure(task_m)

        assert_equal 'object_global', task.properties.object_frame
        assert_equal 'world_global', task.properties.world_frame
    end

    it "does not attempt to use a producer's own output to configure it" do
        producer_task_m = Syskit::TaskContext.new_submodel name: "DataProducer" do
            output_port "transforms", "/base/samples/RigidBodyState"
            transformer do
                transform "object", "world"
                transform_output "transforms", "object" => "world"
                max_latency 0.1
            end
        end
        producer_m =
            producer_task_m
            .use_frames("object" => "a", "world" => "b")
            .transformer { frames "a", "b" }
        task_m =
            data_consumer_m
            .use_frames("object" => "a", "world" => "b")
            .transformer { dynamic_transform producer_m, "a" => "b" }
        task = syskit_stub_deploy_and_configure(task_m)

        producer = task.each_child.first.first
        assert_kind_of producer_task_m, producer
        assert producer.transforms_port.connected_to?(task.dynamic_transformations_port)
    end

    it "rejects inverted transforms when checking for self-produced transforms" do
        producer_task_m = Syskit::TaskContext.new_submodel name: "DataProducer" do
            output_port "transforms", "/base/samples/RigidBodyState"
            transformer do
                transform "object", "world"
                transform_output "transforms", "world" => "object"
                max_latency 0.1
            end
        end
        producer_m =
            producer_task_m
            .use_frames("object" => "a", "world" => "b")
            .transformer { frames "a", "b" }
        task_m =
            data_consumer_m
            .use_frames("object" => "a", "world" => "b")
            .transformer { dynamic_transform producer_m, "b" => "a" }
        task = syskit_stub_deploy_and_configure(task_m)

        producer = task.each_child.first.first
        assert_kind_of producer_task_m, producer
        assert producer.transforms_port.connected_to?(task.dynamic_transformations_port)
    end

    it "propagates data port frame information forward in the dataflow" do
        cmp_m.add self.data_producer_m, as: 'producer'
        cmp_m.add self.data_consumer_m, as: 'consumer'
        cmp_m.producer_child.samples_port.connect_to cmp_m.consumer_child.samples_port

        cmp_m = self.cmp_m.
            use_frames('producer_object' => 'object_global',
                'producer_world' => 'world_global',
                'world' => 'world_global').
            transformer do
                static_transform Eigen::Vector3.new(1, 0, 0),
                    "object_global" => "world_global"
            end
        cmp = syskit_stub_and_deploy(cmp_m)
        assert_equal Hash['world' => 'world_global', 'object' => 'object_global'],
            cmp.consumer_child.selected_frames
    end

    it "propagates transform port frame information forward in the dataflow" do
        cmp_m = Syskit::Composition.new_submodel
        cmp_m.add self.data_producer_m, as: 'producer'
        cmp_m.add self.data_consumer_m, as: 'consumer'
        cmp_m.producer_child.transform_samples_port.connect_to cmp_m.consumer_child.transform_samples_port

        cmp_m = cmp_m.
            use_frames('producer_object' => 'object_global',
                       'producer_world' => 'world_global').
            transformer do
                static_transform Eigen::Vector3.new(1, 0, 0),
                    "object_global" => "world_global"
            end
        cmp = syskit_stub_and_deploy(cmp_m)
        assert_equal Hash['world' => 'world_global', 'object' => 'object_global'],
            cmp.consumer_child.selected_frames
    end

    it "marks invalid chains in the plan with special placeholders, raising only during "\
       "validation" do
        task_m =
            data_consumer_m
            .use_frames("object" => "object_global", "world" => "world_global")
            .transformer { frames "object_global", "world_global" }

        task = syskit_deploy(
            task_m,
            validate_generated_network: false, compute_deployments: false,
            compute_policies: false, validate_deployed_network: false
        )
        placeholder_task, = task.each_child.first
        assert(placeholder_task)
        assert_equal "object", placeholder_task.task_from
        assert_equal "world", placeholder_task.task_to
        assert_equal "object_global", placeholder_task.from
        assert_equal "world_global", placeholder_task.to
        refute_nil placeholder_task.transformer
    end

    it "raises if a chain cannot be resolved" do
        task_m =
            data_consumer_m
            .use_frames("object" => "object_global", "world" => "world_global")
            .transformer { frames "object_global", "world_global" }

        e = assert_raises(Transformer::InvalidChain) do
            syskit_deploy(task_m)
        end

        expected_m = "cannot find a transformation chain to produce object_global => "\
                     "world_global for DataConsumer.* \\\(task-local frames: object => "\
                     "world\\\): no transformation from 'object_global' to "\
                     "'world_global' available"
        assert_match Regexp.new(expected_m), e.message
    end

    it "instanciates dynamic producers" do
        transform_producer_m = self.transform_producer_m
        syskit_stub_requirements(transform_producer_m)
        task_m = self.data_consumer_m.
            use_frames('object' => 'object_global', 'world' => 'world_global').
            transformer do
                dynamic_transform transform_producer_m,
                    "object_global" => "world_global"
            end

        task = syskit_stub_and_deploy(task_m)
        producer = task.child_from_role("transformer_object_global2world_global")
        assert_kind_of transform_producer_m, producer
        assert_equal Hash['producer_object' => 'object_global', 'producer_world' => 'world_global'],
            producer.selected_frames
        assert producer.transform_port.connected_to?(task.dynamic_transformations_port)
    end

    it "instanciates dynamic producers" do
        transform_producer_m = self.transform_producer_m
        syskit_stub_requirements(transform_producer_m)
        task_m = self.data_consumer_m.
            use_frames('object' => 'object_global', 'world' => 'world_global').
            transformer do
                dynamic_transform transform_producer_m,
                    "object_global" => "world_global"
            end

        task = syskit_stub_and_deploy(task_m)
        producer = task.child_from_role("transformer_object_global2world_global")
        assert_kind_of transform_producer_m, producer
        assert_equal Hash['producer_object' => 'object_global', 'producer_world' => 'world_global'],
            producer.selected_frames
        assert producer.transform_port.connected_to?(task.dynamic_transformations_port)
    end

    it "automatically selects producers connected to dedicated ports" do
        transform_producer_m = self.transform_producer_m
        task_m = self.data_consumer_m.
            use_frames('object' => 'object_global', 'world' => 'world_global').
            transformer { frames 'object_global', 'world_global' }
        cmp_m.add transform_producer_m, as: 'producer'
        cmp_m.add task_m, as: 'processor'
        cmp_m.producer_child.transform_port.
            connect_to cmp_m.processor_child.transform_samples_port

        cmp = syskit_stub_and_deploy(cmp_m)
        processor = cmp.processor_child
        assert !cmp.processor_child.dynamic_transformations_port.connected?
    end

    it "accepts specific services as producers" do
        srv_m = Syskit::DataService.new_submodel do
            output_port 'transforms', 'base/samples/RigidBodyState'
        end
        multi_producer_m = transform_producer_m.new_submodel name: 'MultiProducer' do
            output_port 'alternate_transforms', '/base/samples/RigidBodyState'
            transformer do
                transform_output 'alternate_transforms', 'alternate_object' => 'alternate_world'
            end
        end
        multi_producer_m.provides(
            srv_m, { 'transforms' => 'alternate_transforms' }, as: 'test'
        )

        syskit_stub_requirements(multi_producer_m)
        task_m = self.data_consumer_m.
            use_frames('object' => 'object_global', 'world' => 'world_global').
            transformer do
                dynamic_transform multi_producer_m.test_srv,
                    "object_global" => "world_global"
            end

        task = syskit_stub_and_deploy(task_m)
        producer = task.child_from_role("transformer_object_global2world_global")
        assert_kind_of multi_producer_m, producer
        assert producer.alternate_transforms_port.connected_to?(task.dynamic_transformations_port)
        assert_equal Hash['alternate_object' => 'object_global', 'alternate_world' => 'world_global'],
            producer.selected_frames
    end
end

