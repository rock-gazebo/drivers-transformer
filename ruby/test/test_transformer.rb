require 'transformer/test'

module Transformer
    describe Configuration do
        before do
            @manager = Configuration.new
        end

        describe "#rename_frames" do
            describe "frame set" do
                it "applies the renames to the frame set" do
                    @manager.frames 'to_rename'
                    @manager.rename_frames 'to_rename' => 'test'
                    assert_equal ['test'], @manager.frames.to_a
                end
                it "does not modify frames that are not specified" do
                    @manager.frames 'to_rename'
                    @manager.rename_frames Hash.new
                    assert_equal ['to_rename'], @manager.frames.to_a
                end
            end

            describe "static transforms" do
                before do
                    @t = Eigen::Vector3.new(1, 2, 3)
                    @q = Eigen::Quaternion.new(1, 2, 3, 4)
                    @manager.static_transform @t, @q, 'to_rename_src' => 'to_rename_dst'
                end

                it "applies the renames to the static transforms" do
                    @manager.rename_frames 'to_rename_src' => 'src', 'to_rename_dst' => 'dst'
                    transform = @manager.transformation_for 'src', 'dst'
                    assert_equal 'src', transform.from
                    assert_equal 'dst', transform.to
                    assert_equal @t, transform.translation
                    assert_equal @q, transform.rotation
                end
                it "does not modify frames that are not specified" do
                    @manager.rename_frames Hash.new
                    transform = @manager.transformation_for 'to_rename_src', 'to_rename_dst'
                    assert_equal 'to_rename_src', transform.from
                    assert_equal 'to_rename_dst', transform.to
                    assert_equal @t, transform.translation
                    assert_equal @q, transform.rotation
                end
            end

            describe "dynamic transforms" do
                before do
                    @producer = flexmock
                    @producer.should_receive(dup: @producer)
                    @manager.dynamic_transform @producer, 'to_rename_src' => 'to_rename_dst'
                end
                it "applies the renames to the dynamic transforms" do
                    @manager.rename_frames 'to_rename_src' => 'src', 'to_rename_dst' => 'dst'
                    transform = @manager.transformation_for 'src', 'dst'
                    assert_equal 'src', transform.from
                    assert_equal 'dst', transform.to
                    assert_equal @producer, transform.producer
                end
                it "does not modify frames that are not specified" do
                    @manager.rename_frames Hash.new
                    transform = @manager.transformation_for 'to_rename_src', 'to_rename_dst'
                    assert_equal 'to_rename_src', transform.from
                    assert_equal 'to_rename_dst', transform.to
                    assert_equal @producer, transform.producer
                end
            end

            describe "example transforms" do
                before do
                    @t = Eigen::Vector3.new(1, 2, 3)
                    @q = Eigen::Quaternion.new(1, 2, 3, 4)
                    @manager.example_transform @t, @q, 'to_rename_src' => 'to_rename_dst'
                end
                it "applies the renames to the example transforms" do
                    @manager.rename_frames 'to_rename_src' => 'src', 'to_rename_dst' => 'dst'
                    transform = @manager.example_transform_for 'src', 'dst'
                    assert_equal 'src', transform.from
                    assert_equal 'dst', transform.to
                    assert_equal @t, transform.translation
                    assert_equal @q, transform.rotation
                end
                it "does not modify frames that are not specified" do
                    @manager.rename_frames Hash.new
                    transform = @manager.example_transform_for 'to_rename_src', 'to_rename_dst'
                    assert_equal 'to_rename_src', transform.from
                    assert_equal 'to_rename_dst', transform.to
                    assert_equal @t, transform.translation
                    assert_equal @q, transform.rotation
                end
            end
        end
    end

    describe TransformationManager do
        describe "#transformation_chain" do
            attr_reader :trsf, :conf

            before do
                @trsf = Transformer::TransformationManager.new
                @conf = trsf.conf
                conf.frames "a", "b", "c", "d"
            end

            it "resolves a transformation chain" do
                a_b = conf.identity_transform("a" => "b")
                b_c = conf.dynamic_transform("test", "b" => "c")
                c_d = conf.identity_transform("c" => "d")
                chain = trsf.transformation_chain("a", "d")
                assert_equal([a_b, b_c, c_d], chain.links)
                assert_equal([false, false, false], chain.inversions)
            end

            it "inverts dynamic links if needed" do
                a_b = conf.identity_transform("a" => "b")
                c_b = conf.dynamic_transform("test", "c" => "b")
                c_d = conf.identity_transform("c" => "d")
                chain = trsf.transformation_chain("a", "d")
                assert_equal([a_b, c_b, c_d], chain.links)
                assert_equal([false, true, false], chain.inversions)
            end

            it "inverts static links if needed" do
                a_b = conf.identity_transform("a" => "b")
                c_b = conf.identity_transform("c" => "b")
                c_d = conf.identity_transform("c" => "d")
                chain = trsf.transformation_chain("a", "d")
                assert_equal([a_b, c_b, c_d], chain.links)
                assert_equal([false, true, false], chain.inversions)
            end

            it "resolves the shortest path" do
                a_b = conf.identity_transform("a" => "b")
                c_b = conf.identity_transform("c" => "b")
                c_d = conf.identity_transform("c" => "d")
                a_c = conf.identity_transform("a" => "c")
                chain = trsf.transformation_chain("a", "d")
                assert_equal([a_c, c_d], chain.links)
                assert_equal([false, false], chain.inversions)
            end

            it "rejects static transformations that have a nil producer "\
               "in additional_transforms" do
                conf.identity_transform("a" => "b")
                conf.identity_transform("b" => "c")
                conf.identity_transform("c" => "d")

                # Verify that the configuration does resolve normally
                trsf.transformation_chain("a", "d")
                assert_raises(TransformationNotFound) do
                    trsf.transformation_chain("a", "d", { %w[b c] => nil })
                end
            end

            it "rejects static transformations that have a nil producer "\
               "in additional_transforms, even if specified inverted" do
                conf.identity_transform("a" => "b")
                conf.identity_transform("b" => "c")
                conf.identity_transform("c" => "d")

                # Verify that the configuration does resolve normally
                trsf.transformation_chain("a", "d")
                assert_raises(TransformationNotFound) do
                    trsf.transformation_chain("a", "d", { %w[c b] => nil })
                end
            end

            it "rejects dynamic transformations that have a nil producer "\
               "in additional_transforms" do
                conf.identity_transform( "a" => "b")
                conf.dynamic_transform("test", "b" => "c")
                conf.identity_transform( "c" => "d")

                # Verify that the configuration does resolve normally
                trsf.transformation_chain("a", "d")
                assert_raises(TransformationNotFound) do
                    trsf.transformation_chain("a", "d", { %w[b c] => nil })
                end
            end

            it "rejects dynamic transformations that have a nil producer "\
               "in additional_transforms, even if specified inverted" do
                conf.identity_transform( "a" => "b")
                conf.dynamic_transform("test", "b" => "c")
                conf.identity_transform( "c" => "d")

                # Verify that the configuration does resolve normally
                trsf.transformation_chain("a", "d")
                assert_raises(TransformationNotFound) do
                    trsf.transformation_chain("a", "d", { %w[c b] => nil })
                end
            end
        end

        describe "#dup" do
            it "isolates the sets of the receiver from the new object" do
                trsf = Transformer::TransformationManager.new
                conf = trsf.conf
                conf.static_transform Eigen::Vector3.new(0, 0, 0), "body" => "servo_low"
                conf.dynamic_transform "dynamixel", "servo_high" => "servo_low"
                copy = conf.dup
                conf.clear
                assert_equal 2, copy.transforms.size
                assert_equal 3, copy.frames.size
            end
        end
    end
end
