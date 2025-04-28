"""snake_case

Revision ID: f7f044643953
Revises: 
Create Date: 2025-04-25 17:04:16.638233

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision: str = 'f7f044643953'
down_revision: Union[str, None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # 칼럼 이름 변경만 수행
    op.alter_column('routines', 'routineId', new_column_name='routine_id')
    op.alter_column('routines', 'startTime', new_column_name='start_time')
    op.alter_column('routines', 'routineType', new_column_name='routine_type')
    op.alter_column('routines', 'isWork', new_column_name='is_work')
    op.alter_column('routines', 'repeatDays', new_column_name='repeat_days')

def downgrade() -> None:
    """Downgrade schema."""
    # 칼럼 이름을 원래대로 되돌림
    op.alter_column('users', 'user_id', new_column_name='userId')
    op.alter_column('devices', 'user_id', new_column_name='userId')
    op.alter_column('routines', 'user_id', new_column_name='userId')
    op.alter_column('routines', 'routine_id', new_column_name='routineId')
    op.alter_column('routines', 'start_time', new_column_name='startTime')
    op.alter_column('routines', 'routine_type', new_column_name='routineType')
    op.alter_column('routines', 'is_work', new_column_name='isWork')
    op.alter_column('routines', 'repeat_days', new_column_name='repeatDays')