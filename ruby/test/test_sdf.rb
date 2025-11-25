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

        it "creates an example transform between the model's canonical link frame and the world using the model pose" do
            conf.load_sdf('model://model_with_only_root_links')
            tr = conf.example_transform_for('m::root_link', 'w')
            assert_eigen_approx Eigen::Vector3.new(1, 3, 5), tr.translation
            assert Eigen::Quaternion.from_angle_axis(1, Eigen::Vector3.UnitZ).approx?(tr.rotation)
        end

        it "creates an static transform between the model's canonical link and the world using the model pose if the model is static" do
            conf.load_sdf('model://static_model')
            tr = conf.transform_for('m::root_link', 'w')
            assert_eigen_approx Eigen::Vector3.new(1, 3, 5), tr.translation
            assert Eigen::Quaternion.from_angle_axis(1, Eigen::Vector3.UnitZ).approx?(tr.rotation)
        end

        it "creates a static transform between canonical links and the model" do
            conf.load_sdf('model://model_with_only_root_links')
            tr = conf.transformation_for('m::root_link', 'm')
            assert_equal Eigen::Vector3.Zero, tr.translation
            assert_equal Eigen::Quaternion.Identity, tr.rotation
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

        it "declares even frames that are not linked to the root frame through joints" do
            load_sdf(<<-EOSDF)
            <sdf><world name="w">
            <model name="m"><link name="root" />
              <link name="l" />
              <model name="subm"><link name="l" />
                <model name="subsubm"><link name="l" />
                </model>
              </model>
            </model></world></sdf>
            EOSDF
            assert conf.has_frame?('m::l')
            assert conf.has_frame?('m::subm::l')
            assert conf.has_frame?('m::subm::subsubm::l')
        end

        describe "#sdf_add_root_model" do
            it "declares the frame of the root model" do
                load_sdf(<<-EOSDF)
                <sdf><world name="w"><model name="m" /></world></sdf>
                EOSDF

                assert conf.frame?("m")
            end

            it "declares the static transform between the root model and its " \
               "canonical link" do
                load_sdf(<<-EOSDF)
                <sdf><world name="w">
                    <model name="m">
                        <link name="canonical" />
                    </model>
                </world></sdf>
                EOSDF

                assert conf.frame?("m::canonical")
                assert_equal Eigen::Isometry3.Identity,
                             conf.resolve_static_chain("m", "m::canonical")
            end
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
            describe "a joint with the world link as child" do
                before do
                    load_sdf(<<-EOSDF)
                    <sdf><world name="w"><model name="m">
                         <pose>1 2 3 0 0 0</pose>
                         <link name="root" />
                         <link name="l">
                           <pose>2 3 4 0 0 0</pose>
                         </link>
                         <joint name="root_to_l" type="fixed">
                            <parent>root</parent>
                            <child>l</child>
                         </joint>
                         <joint name="world_link_as_child" type="fixed">
                            <parent>l</parent>
                            <child>world</child>
                         </joint>
                    </model></world></sdf>
                    EOSDF
                end

                it "creates a transform between the world frame and the parent link" do
                    t = conf.transformation_for 'w', 'm::l'
                    assert_kind_of StaticTransform, t
                end
                it "uses the world pose of the link to setup the transformation" do
                    t = conf.transformation_for 'w', 'm::l'
                    assert_eigen_approx Eigen::Vector3.new(-3, -5, -7), t.translation
                    assert_equal Eigen::Quaternion.Identity, t.rotation
                end
                it "does not setup a transform between the model and the world" do
                    refute conf.has_transform?('w', 'm::root')
                    refute conf.has_transform?('m::root', 'w')
                end
            end

            describe "a joint with the world link as parent" do
                before do
                    load_sdf(<<-EOSDF)
                    <sdf><world name="w"><model name="m">
                         <pose>1 2 3 0 0 0</pose>
                         <link name="root" />
                         <link name="l">
                           <pose>2 3 4 0 0 0</pose>
                         </link>
                         <joint name="root_to_l" type="fixed">
                            <parent>root</parent>
                            <child>l</child>
                         </joint>
                         <joint name="world_link_as_child" type="fixed">
                            <parent>world</parent>
                            <child>l</child>
                         </joint>
                    </model></world></sdf>
                    EOSDF
                end

                it "creates a transform between the world frame and another link in case a joint has 'world' as child" do
                    t = conf.transformation_for 'm::l', 'w'
                    assert_kind_of StaticTransform, t
                end
                it "uses the world pose of the link to setup the transformation" do
                    t = conf.transformation_for 'm::l', 'w'
                    assert_eigen_approx Eigen::Vector3.new(3, 5, 7), t.translation
                    assert_equal Eigen::Quaternion.Identity, t.rotation
                end
                it "does not setup a transform between the model and the world" do
                    refute conf.has_transform?('w', 'm::root')
                    refute conf.has_transform?('m::root', 'w')
                end
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
            it "applies the pose of the submodel onto the links" do
                load_sdf(<<-EOSDF)
                <sdf><world name="w">
                <model name="m">
                   <pose>1 2 3 0 0 0</pose>
                   <link name="l" />
                   <model name="subm">
                      <pose>2 3 4 0 0 0</pose>
                      <link name="l">
                        <pose>3 4 5 0 0 0</pose>
                      </link>
                   </model>
                   <joint name="j" type="fixed">
                      <parent>l</parent>
                      <child>subm::l</child>
                   </joint>
                </model></world></sdf>
                EOSDF
                t = conf.transformation_for('m::subm::l', 'm::l')
                assert_equal Eigen::Vector3.new(5, 7, 9), t.translation
            end
            it "applies the pose of the submodels recursively" do
                load_sdf(<<-EOSDF)
                <sdf><world name="w">
                <model name="m">
                   <pose>1 2 3 0 0 0</pose>
                   <link name="l" />
                   <model name="subm">
                      <pose>2 3 4 0 0 0</pose>
                      <model name="subm2">
                         <link name="l">
                           <pose>3 4 5 0 0 0</pose>
                         </link>
                      </model>
                   </model>
                   <joint name="j" type="fixed">
                     <parent>l</parent>
                     <child>subm::subm2::l</child>
                   </joint>
                </model></world></sdf>
                EOSDF
               t = conf.transformation_for('m::subm::subm2::l', 'm::l')
               assert_eigen_approx Eigen::Vector3.new(5, 7, 9), t.translation
            end
        end

        def load_sdf(sdf_xml)
            sdf = ::SDF::Root.from_xml_string(sdf_xml)
            conf.parse_sdf_root(sdf)
        end
    end
end
