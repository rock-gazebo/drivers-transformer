require 'transformer/test'
require 'transformer/sdf'

module Transformer
    describe SDF do
        attr_reader :conf

        before do
            model_dir = File.expand_path(File.join('data', 'sdf'), File.dirname(__FILE__))
            @orig_model_path = ::SDF::XML.model_path.dup
            ::SDF::XML.model_path << model_dir
            @conf = Configuration.new
        end

        after do
            ::SDF::XML.model_path = @orig_model_path
        end

        it "loads a root model just fine" do
            conf.load_sdf('model://root_model')
            assert conf.has_frame?('root_model_name')
        end

        it "prefixes frames hierarchically" do
            conf.load_sdf('model://model_within_a_world')
            assert_equal %w{root_model_name world_name},
                conf.frames.to_a.sort
        end

        it "creates an example transform between the model frame and the world using the model pose" do
            conf.load_sdf('model://model_with_only_root_links')
            tr = conf.example_transform_for('m', 'w')
            assert Eigen::Vector3.new(0, 1, 2).approx?(tr.translation)
            assert Eigen::Quaternion.from_angle_axis(1, Eigen::Vector3.UnitZ).approx?(tr.rotation)
        end

        it "creates an static transform between the model frame and the world using the model pose if the model is static" do
            conf.load_sdf('model://static_model')
            tr = conf.transform_for('m', 'w')
            assert Eigen::Vector3.new(0, 1, 2).approx?(tr.translation)
            assert Eigen::Quaternion.from_angle_axis(1, Eigen::Vector3.UnitZ).approx?(tr.rotation)
        end

        it "creates a static transform between root links and the model" do
            conf.load_sdf('model://model_with_only_root_links')
            tr = conf.transformation_for('m::root_link', 'm')
            assert Eigen::Vector3.new(1, 2, 3).
                approx?(tr.translation)
            assert Eigen::Quaternion.from_angle_axis(2, Eigen::Vector3.UnitZ).
                approx?(tr.rotation)
        end

        it "creates static transforms between the links and the joints" do
            conf.load_sdf('model://model_with_child_links')
            tr = conf.transformation_for('m::j_post', 'm::child_link')
            assert Eigen::Vector3.new(1, 2, 3).
                approx?(tr.translation)
            assert Eigen::Quaternion.from_angle_axis(2, Eigen::Vector3.UnitZ).
                approx?(tr.rotation)
        end

        it "creates dynamic transforms between root links and child links if a transformation producer is given" do
            recorder = flexmock
            recorder.should_receive(:call).with('w::m::j').once
            conf.load_sdf('model://model_with_child_links') do |joint|
                recorder.call(joint.full_name)
                'producer'
            end
            tr = conf.transformation_for('m::j_post', 'm::j_pre')
            assert 'producer', tr.producer
        end


        describe "example transform for revolute joints" do
            it "provides an example transform that matches the middle of the axis range " do
                conf.load_sdf('model://model_with_child_links')
                # Because of the flag, the rotation axis is (0, -1, 0) instead
                # of (1, 0, 0)
                tr = conf.example_transform_for 'm::j_post', 'm::j_pre'
                assert_equal Eigen::Vector3.Zero, tr.translation
                assert Eigen::Quaternion.from_angle_axis(0.1, Eigen::Vector3.UnitX).approx?(tr.rotation)
            end
            it "modifies the rotation axis according to use_parent_model_frame" do
                conf.load_sdf('model://joint_with_use_parent_model_frame')
                # Because of the flag, the rotation axis is (0, -1, 0) instead
                # of (1, 0, 0)
                tr = conf.example_transform_for 'm::j_post', 'm::j_pre'
                assert_equal Eigen::Vector3.Zero, tr.translation
                assert Eigen::Quaternion.from_angle_axis(0.1, -Eigen::Vector3.UnitY).approx?(tr.rotation, 1e-6)
            end
        end

        describe "handling of the special 'world' link" do
            it "creates a transform between the world frame and another link in case a joint has 'world' as parent" do
                conf.load_sdf('model://joint_with_world_link')
                t = conf.transformation_for 'w', 'root::parent_of_world'
                assert_equal Eigen::Vector3.Zero, t.translation
                assert_equal Eigen::Quaternion.Identity, t.rotation
            end
            it "creates a transform between the world frame and another link in case a joint has 'world' as child" do
                conf.load_sdf('model://joint_with_world_link')
                t = conf.transformation_for 'root::child_of_world', 'w'
                assert_equal Eigen::Vector3.Zero, t.translation
                assert_equal Eigen::Quaternion.Identity, t.rotation
            end
        end

        describe "handling of models in models" do
            it "creates properly namespaced frames" do
                conf.load_sdf('model://joint_with_world_link')
                assert conf.has_frame?('root::submodel::l')
            end
            it "handles namespaces properly when creating cross-model joints" do
                conf.load_sdf('model://joint_with_world_link')
                t = conf.transformation_for('root::l', 'root::submodel::l')
                assert_equal 'root::l', t.from
                assert_equal 'root::submodel::l', t.to
            end
        end
    end
end
